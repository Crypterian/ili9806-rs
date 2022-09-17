#![no_std]

//! ILI9806 Display Driver
//!
//! ### Usage
//!
//! To control the display you need to set up:
//!
//! * Interface for communicating with display ([display-interface-spi crate] for SPI)
//! * Configuration (reset pin, delay, orientation and size) for display
//!
//! ```ignore
//! let iface = SPIInterface::new(spi, dc, cs);
//!
//! let mut display = Ili9806::new(
//!     iface,
//!     reset_gpio,
//!     &mut delay,
//!     Orientation::Landscape,
//!     ili9806::DisplaySize480x480,
//! )
//! .unwrap();
//!
//! display.clear(Rgb565::RED).unwrap()
//! ```
//!
//! [display-interface-spi crate]: https://crates.io/crates/display-interface-spi
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::digital::v2::OutputPin;

use core::iter::once;
use display_interface::DataFormat::{U16BEIter, U8Iter};
use display_interface::WriteOnlyDataCommand;

#[cfg(feature = "graphics")]
mod graphics_core;

pub use embedded_hal::spi::MODE_0 as SPI_MODE;

pub use display_interface::DisplayError;

type Result<T = (), E = DisplayError> = core::result::Result<T, E>;

/// Trait that defines display size information
pub trait DisplaySize {
    /// Width in pixels
    const WIDTH: usize;
    /// Height in pixels
    const HEIGHT: usize;
}

/// Generic display size of 480x480 pixels
pub struct DisplaySize480x480;

impl DisplaySize for DisplaySize480x480 {
    const WIDTH: usize = 480;
    const HEIGHT: usize = 480;
}

/// For quite a few boards (ESP32-S2-Kaluga-1, M5Stack, M5Core2 and others),
/// the ILI9341 initialization command arguments are slightly different
///
/// This trait provides the flexibility for users to define their own
/// initialization command arguments suitable for the particular board they are using
pub trait Mode {
    fn mode(&self) -> u8;

    fn is_landscape(&self) -> bool;
}

/// The default implementation of the Mode trait from above
/// Should work for most (but not all) boards
pub enum Orientation {
    Portrait,
    PortraitFlipped,
    Landscape,
    LandscapeFlipped,
}

impl Mode for Orientation {
    fn mode(&self) -> u8 {
        match self {
            Self::Portrait => 0x40 | 0x08,
            Self::Landscape => 0x20 | 0x08,
            Self::PortraitFlipped => 0x80 | 0x08,
            Self::LandscapeFlipped => 0x40 | 0x80 | 0x20 | 0x08,
        }
    }

    fn is_landscape(&self) -> bool {
        match self {
            Self::Landscape | Self::LandscapeFlipped => true,
            Self::Portrait | Self::PortraitFlipped => false,
        }
    }
}

/// Specify state of specific mode of operation
pub enum ModeState {
    On,
    Off,
}

/// There are two method for drawing to the screen:
/// [Ili9806::draw_raw_iter] and [Ili9806::draw_raw_slice]
///
/// In both cases the expected pixel format is rgb565.
///
/// The hardware makes it efficient to draw rectangles on the screen.
///
/// What happens is the following:
///
/// - A drawing window is prepared (with the 2 opposite corner coordinates)
/// - The starting point for drawint is the top left corner of this window
/// - Every pair of bytes received is intepreted as a pixel value in rgb565
/// - As soon as a pixel is received, an internal counter is incremented,
///   and the next word will fill the next pixel (the adjacent on the right, or
///   the first of the next row if the row ended)
pub struct Ili9806<IFACE, RESET> {
    interface: IFACE,
    reset: RESET,
    width: usize,
    height: usize,
    landscape: bool,
}

impl<IFACE, RESET> Ili9806<IFACE, RESET>
where
    IFACE: WriteOnlyDataCommand,
    RESET: OutputPin,
{
    pub fn new<DELAY, SIZE, MODE>(
        interface: IFACE,
        reset: RESET,
        delay: &mut DELAY,
        mode: MODE,
        _display_size: SIZE,
    ) -> Result<Self>
    where
        DELAY: DelayMs<u8>,
        SIZE: DisplaySize,
        MODE: Mode,
    {
        let mut ili9806 = Ili9806 {
            interface,
            reset,
            width: SIZE::WIDTH,
            height: SIZE::HEIGHT,
            landscape: false,
        };

        // Do hardware reset by holding reset low for at least 10us
        ili9806.reset.set_low().map_err(|_| DisplayError::RSError)?;
        let _ = delay.delay_ms(1);
        // Set high for normal operation
        ili9806
            .reset
            .set_high()
            .map_err(|_| DisplayError::RSError)?;

        // Wait 5ms after reset before sending commands
        // and 120ms before sending Sleep Out
        let _ = delay.delay_ms(5);

        // Do software reset
        ili9806.command(Command::SoftwareReset, &[])?;

        // Wait 5ms after reset before sending commands
        // and 120ms before sending Sleep Out
        let _ = delay.delay_ms(120);

        ili9806.set_orientation(mode)?;

        // Set pixel format to 16 bits per pixel
        ili9806.command(Command::PixelFormatSet, &[0x55])?;

        ili9806.sleep_mode(ModeState::Off)?;

        // Wait 5ms after Sleep Out before sending commands
        let _ = delay.delay_ms(5);

        ili9806.display_mode(ModeState::On)?;

        Ok(ili9806)
    }
}

impl<IFACE, RESET> Ili9806<IFACE, RESET>
where
    IFACE: WriteOnlyDataCommand,
{
    fn command(&mut self, cmd: Command, args: &[u8]) -> Result {
        self.interface.send_commands(U8Iter(&mut once(cmd as u8)))?;
        self.interface.send_data(U8Iter(&mut args.iter().cloned()))
    }

    fn write_iter<I: IntoIterator<Item = u16>>(&mut self, data: I) -> Result {
        self.command(Command::MemoryWrite, &[])?;
        self.interface.send_data(U16BEIter(&mut data.into_iter()))
    }

    fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) -> Result {
        self.command(
            Command::ColumnAddressSet,
            &[
                (x0 >> 8) as u8,
                (x0 & 0xff) as u8,
                (x1 >> 8) as u8,
                (x1 & 0xff) as u8,
            ],
        )?;
        self.command(
            Command::PageAddressSet,
            &[
                (y0 >> 8) as u8,
                (y0 & 0xff) as u8,
                (y1 >> 8) as u8,
                (y1 & 0xff) as u8,
            ],
        )
    }

    /// Configures the screen for hardware-accelerated vertical scrolling.
    pub fn configure_vertical_scroll(
        &mut self,
        fixed_top_lines: u16,
        fixed_bottom_lines: u16,
    ) -> Result<Scroller> {
        let height = if self.landscape {
            self.width
        } else {
            self.height
        } as u16;
        let scroll_lines = height as u16 - fixed_top_lines - fixed_bottom_lines;

        self.command(
            Command::VerticalScrollDefine,
            &[
                (fixed_top_lines >> 8) as u8,
                (fixed_top_lines & 0xff) as u8,
                (scroll_lines >> 8) as u8,
                (scroll_lines & 0xff) as u8,
                (fixed_bottom_lines >> 8) as u8,
                (fixed_bottom_lines & 0xff) as u8,
            ],
        )?;

        Ok(Scroller::new(fixed_top_lines, fixed_bottom_lines, height))
    }

    pub fn scroll_vertically(&mut self, scroller: &mut Scroller, num_lines: u16) -> Result {
        scroller.top_offset += num_lines;
        if scroller.top_offset > (scroller.height - scroller.fixed_bottom_lines) {
            scroller.top_offset = scroller.fixed_top_lines
                + (scroller.top_offset + scroller.fixed_bottom_lines - scroller.height)
        }

        self.command(
            Command::VerticalScrollAddr,
            &[
                (scroller.top_offset >> 8) as u8,
                (scroller.top_offset & 0xff) as u8,
            ],
        )
    }

    /// Draw a rectangle on the screen, represented by top-left corner (x0, y0)
    /// and bottom-right corner (x1, y1).
    ///
    /// The border is included.
    ///
    /// This method accepts an iterator of rgb565 pixel values.
    ///
    /// The iterator is useful to avoid wasting memory by holding a buffer for
    /// the whole screen when it is not necessary.
    pub fn draw_raw_iter<I: IntoIterator<Item = u16>>(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
        data: I,
    ) -> Result {
        self.set_window(x0, y0, x1, y1)?;
        self.write_iter(data)
    }

    /// Draw a rectangle on the screen, represented by top-left corner (x0, y0)
    /// and bottom-right corner (x1, y1).
    ///
    /// The border is included.
    ///
    /// This method accepts a raw buffer of words that will be copied to the screen
    /// video memory.
    ///
    /// The expected format is rgb565.
    pub fn draw_raw_slice(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, data: &[u16]) -> Result {
        self.draw_raw_iter(x0, y0, x1, y1, data.iter().copied())
    }

    /// Change the orientation of the screen
    pub fn set_orientation<MODE>(&mut self, mode: MODE) -> Result
    where
        MODE: Mode,
    {
        self.command(Command::MemoryAccessControl, &[mode.mode()])?;

        if self.landscape ^ mode.is_landscape() {
            core::mem::swap(&mut self.height, &mut self.width);
        }
        self.landscape = mode.is_landscape();
        Ok(())
    }

    /// Fill entire screen with specfied color u16 value
    pub fn clear_screen(&mut self, color: u16) -> Result {
        let color = core::iter::repeat(color).take(self.width * self.height);
        self.draw_raw_iter(0, 0, self.width as u16, self.height as u16, color)
    }

    /// Control the screen sleep mode:
    pub fn sleep_mode(&mut self, mode: ModeState) -> Result {
        match mode {
            ModeState::On => self.command(Command::SleepModeOn, &[]),
            ModeState::Off => self.command(Command::SleepModeOff, &[]),
        }
    }

    /// Control the screen display mode
    pub fn display_mode(&mut self, mode: ModeState) -> Result {
        match mode {
            ModeState::On => self.command(Command::DisplayOn, &[]),
            ModeState::Off => self.command(Command::DisplayOff, &[]),
        }
    }

    /// Invert the pixel color on screen
    pub fn invert_mode(&mut self, mode: ModeState) -> Result {
        match mode {
            ModeState::On => self.command(Command::InvertOn, &[]),
            ModeState::Off => self.command(Command::InvertOff, &[]),
        }
    }

    /// Idle mode reduces the number of colors to 8
    pub fn idle_mode(&mut self, mode: ModeState) -> Result {
        match mode {
            ModeState::On => self.command(Command::IdleModeOn, &[]),
            ModeState::Off => self.command(Command::IdleModeOff, &[]),
        }
    }

    /// Set display brightness to the value between 0 and 255
    pub fn brightness(&mut self, brightness: u8) -> Result {
        self.command(Command::SetBrightness, &[brightness])
    }

    /// Set adaptive brightness value equal to [AdaptiveBrightness]
    pub fn content_adaptive_brightness(&mut self, value: AdaptiveBrightness) -> Result {
        self.command(Command::ContentAdaptiveBrightness, &[value as _])
    }

    /// Configure [FrameRateClockDivision] and [FrameRate] in normal mode
    pub fn normal_mode_frame_rate(
        &mut self,
        clk_div: FrameRateClockDivision,
        frame_rate: FrameRate,
    ) -> Result {
        self.command(
            Command::NormalModeFrameRate,
            &[clk_div as _, frame_rate as _],
        )
    }

    /// Configure [FrameRateClockDivision] and [FrameRate] in idle mode
    pub fn idle_mode_frame_rate(
        &mut self,
        clk_div: FrameRateClockDivision,
        frame_rate: FrameRate,
    ) -> Result {
        self.command(Command::IdleModeFrameRate, &[clk_div as _, frame_rate as _])
    }
}

impl<IFACE, RESET> Ili9806<IFACE, RESET> {
    /// Get the current screen width. It can change based on the current orientation
    pub fn width(&self) -> usize {
        self.width
    }

    /// Get the current screen heighth. It can change based on the current orientation
    pub fn height(&self) -> usize {
        self.height
    }
}

/// Scroller must be provided in order to scroll the screen. It can only be obtained
/// by configuring the screen for scrolling.
pub struct Scroller {
    top_offset: u16,
    fixed_bottom_lines: u16,
    fixed_top_lines: u16,
    height: u16,
}

impl Scroller {
    fn new(fixed_top_lines: u16, fixed_bottom_lines: u16, height: u16) -> Scroller {
        Scroller {
            top_offset: fixed_top_lines,
            fixed_top_lines,
            fixed_bottom_lines,
            height,
        }
    }
}

/// Available Adaptive Brightness values
pub enum AdaptiveBrightness {
    Off = 0x00,
    UserInterfaceImage = 0x01,
    StillPicture = 0x02,
    MovingImage = 0x03,
}

/// Available frame rate in Hz
pub enum FrameRate {
    FrameRate119 = 0x10,
    FrameRate112 = 0x11,
    FrameRate106 = 0x12,
    FrameRate100 = 0x13,
    FrameRate95 = 0x14,
    FrameRate90 = 0x15,
    FrameRate86 = 0x16,
    FrameRate83 = 0x17,
    FrameRate79 = 0x18,
    FrameRate76 = 0x19,
    FrameRate73 = 0x1a,
    FrameRate70 = 0x1b,
    FrameRate68 = 0x1c,
    FrameRate65 = 0x1d,
    FrameRate63 = 0x1e,
    FrameRate61 = 0x1f,
}

/// Frame rate clock division
pub enum FrameRateClockDivision {
    Fosc = 0x00,
    FoscDiv2 = 0x01,
    FoscDiv4 = 0x02,
    FoscDiv8 = 0x03,
}

#[derive(Clone, Copy)]
enum Command {
    NOP = 0x00,
    SoftwareReset = 0x01,
    ReadDisplayIdentificationInformation = 0x04,
    ReadNumberOfErrorsOnDSI = 0x05,
    GetRedChannel = 0x06,
    GetGreenChannel = 0x07,
    GetBlueChannel = 0x08,
    ReadDisplayStatus = 0x09,
    ReadDisplayPowerMode = 0x0a,
    ReadDisplayMADCTL = 0x0b,
    ReadDisplayPixelFormat = 0x0c,
    ReadDisplayImageMode = 0x0d,
    ReadDisplaySignalMode = 0x0e,
    ReadDisplaySelfDiagnosticResult = 0x0f,
    SleepModeOn = 0x10,
    SleepModeOff = 0x11,
    PartialModeOn = 0x12,
    NormalDisplayModeOn = 0x13,

    InvertOff = 0x20,
    InvertOn = 0x21,
    AllPixelsOff = 0x22,
    AllPixelsOn = 0x23,
    GammeSet = 0x26,
    DisplayOff = 0x28,
    DisplayOn = 0x29,
    ColumnAddressSet = 0x2a,
    PageAddressSet = 0x2b,
    MemoryWrite = 0x2c,
    MemoryRead = 0x2e,

    PartialArea = 0x30,
    VerticalScrollDefine = 0x33,
    TearingEffectLineOff = 0x34,
    TearingEffectLineOn = 0x35,
    MemoryAccessControl = 0x36,
    VerticalScrollAddr = 0x37,
    IdleModeOff = 0x38,
    IdleModeOn = 0x39,
    PixelFormatSet = 0x3a,
    MemoryWriteContinue = 0x3c,
    MemoryReadContinue = 0x3e,

    WriteTearScanLine = 0x44,
    ReadScanLine = 0x45,

    SetBrightness = 0x51,
    ReadDisplayBrightness = 0x52,
    WriteCTRLDisplayValue = 0x53,
    ReadCTRLDisplayValue = 0x54,
    ContentAdaptiveBrightness = 0x55,
    ReadContentAdaptiveBrightness = 0x56,
    WriteCBACMinimumBrightness = 0x5e,
    ReadCBACMinimumBrightness = 0x5f,

    ReadAutomaticBrightnessControlSelfDiagnosticResult = 0x68,

    ReadBlackWhiteLowBits = 0x70,
    ReadBkx = 0x71,
    ReadBky = 0x72,
    ReadWx = 0x73,
    ReadWy = 0x74,
    ReadRedGreenLowBits = 0x75,
    ReadRx = 0x76,
    ReadRy = 0x77,
    ReadGx = 0x78,
    ReadGy = 0x79,
    ReadBlueALowBits = 0x7a,
    ReadBx = 0x7b,
    ReadBy = 0x7c,
    ReadAx = 0x7d,
    ReadAy = 0x7e,

    ReadDDBStart = 0xa1,
    ReadDDBContinue = 0xa8,
    ReadFirstChecksum = 0xaa,
    ReadContinueChecksum = 0xaf,

    InterfaceModeControl = 0xb0,
    NormalModeFrameRate = 0xb1,
    IdleModeFrameRate = 0xb2,
    PartialModeFrameRate = 0xb3,
    DisplayInversionControl = 0xb4,
    BlankinPorchControl = 0xb5,
    DisplayFunctionControl = 0xb6,
    EnteryModeSet = 0xb7,
    DBITypeBInterfaceSetting = 0xb8,
    PanelControl = 0xb9,
    SPIInterfaceSetting = 0xba,

    PowerControl1 = 0xc0,
    PowerControl2 = 0xc1,
    PowerControl3 = 0xc2,
    VCOMControl1 = 0xc7,
    BacklightControl1 = 0xc8,
    BacklightControl2 = 0xc9,
    BacklightControl3 = 0xca,
    BacklightControl4 = 0xcb,
    BacklightControl5 = 0xcc,
    BacklightControl6 = 0xcd,
    BacklightControl7 = 0xce,
    BacklightControl8 = 0xcf,

    NVMemoryWrite = 0xd0,
    NVMemoryProtectionKey = 0xd1,
    NVMemoryStatusRead = 0xd2,
    ReadDeviceCode = 0xd3,
    ReadID1 = 0xda,
    ReadID2 = 0xdb,
    ReadID3 = 0xdc,
    EngineeringSetting = 0xdf,

    PositiveGammaControl = 0xe0,
    NegativeGammaCorrection = 0xe1,
    DigitalGammaControl1 = 0xe2,
    DigitalGammaControl2 = 0xe3,
    Digital3GammaEnable = 0xea,
    VoltageMeasurementSet = 0xed,

    PanelTimingControl1 = 0xf1,
    PanelTimingControl2 = 0xf2,
    DVDDVoltageSetting = 0xf3,
    PowerControl5 = 0xf5,
    PanelResolutionSelectionSet = 0xf7,
    ReadEXTCCommandInSPIMode = 0xfb,
    LVGLVoltageSetting = 0xfc,
    ExternalPowerSelectionSet = 0xfd,
    EXTCCommandSetEnableRegister = 0xff,
}

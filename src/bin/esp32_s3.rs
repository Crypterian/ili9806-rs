#![no_main]
#![no_std]

use core::fmt::Write;

use display_interface_spi::SPIInterface;
use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyle},
    pixelcolor::Rgb565,
    prelude::*,
    text::{Alignment, Text},
};

use embedded_hal::{
    digital::v2::PinState,
    spi::{Mode, Phase, Polarity, SpiMode},
};
use esp32s3_hal::{
    clock::ClockControl, gpio::IO, i2c::I2C, pac::Peripherals, prelude::*, spi::Spi,
    timer::TimerGroup, Rtc, Serial,
};
use esp_backtrace as _;
use esp_hal_common::OutputPin;
use ili9806::{DisplaySize480x480, Ili9806, Orientation};
use xtensa_lx_rt::entry;

#[derive(Default)]
pub struct NoPin;

impl OutputPin for NoPin {
    type Error = ();

    fn set_low(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_high(&mut self) -> Result<(), Self::Error> {
        Ok(())
    }
    fn set_state(&mut self, _state: PinState) -> Result<(), Self::Error> {
        Ok(())
    }
}

#[entry]
fn main() -> ! {
    let peripherals = Peripherals::take().unwrap();
    let mut system = peripherals.DPORT.split();
    let clocks = ClockControl::boot_defaults(system.clock_control).freeze();

    let timer_group0 = TimerGroup::new(peripherals.TIMG0, &clocks);
    let mut timer0 = timer_group0.timer0;
    let mut wdt = timer_group0.wdt;
    let mut serial0 = Serial::new(peripherals.UART0);
    let mut rtc = Rtc::new(peripherals.RTC_CNTL);

    // Disable watchdog timer
    wdt.disable();
    rtc.rwdt.disable();

    let io = IO::new(peripherals.GPIO, peripherals.IO_MUX);

    writeln!(serial0, "Enabling peripheral!").unwrap();

    /*
     *  The Ili9806 driver
     */

    let reset = io.pins.gpio5.into_alternate_1();
    let sclk = io.pins.gpio45.into_alternate_1();
    let mosi = io.pins.gpio48.into_alternate_1().into_push_pull_output();
    let miso = NoPin;
    let cs = io.pins.gpio38.into_push_pull_output();
    let dc = io.pins.gpio4.into_push_pull_output(); //TODO: FIND WHICH PIN TO USE

    let mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };

    let spi = Spi::new(
        peripherals.SPI0,
        sclk,
        mosi,
        miso,
        cs,
        2u32.MHz(),
        mode,
        &mut system.peripheral_clock_control,
        &clocks,
    );

    let gpioa = io.pins.gpio0;
    let gpiob = io.pins.gpio1;

    let spi_iface = SPIInterface::new(spi, dc, cs);

    let mut delay = peripherals.TIMG1.delay_us(&clocks);

    let mut lcd = Ili9806::new(
        spi_iface,
        reset,
        &mut delay,
        Orientation::PortraitFlipped,
        DisplaySize480x480,
    )
    .unwrap();

    timer0.start(5u64.secs());

    writeln!(serial0, "Starting timer!").unwrap();

    // Create a new character style
    let style = MonoTextStyle::new(&FONT_6X10, Rgb565::RED);

    // Create a text at position (20, 30) and draw it using the previously defined style
    Text::with_alignment(
        "First line\nSecond line",
        Point::new(20, 30),
        style,
        Alignment::Center,
    )
    .draw(&mut lcd)
    .unwrap();

    loop {}
}

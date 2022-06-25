//! Blinks an LED
//!
//! This assumes that a LED is connected to pc13 as is the case on the blue pill board.
//!
//! Note: Without additional hardware, PC13 should not be used to drive an LED, see page 5.1.2 of
//! the reference manual for an explanation. This is not an issue on the blue pill.

#![deny(unsafe_code)]
#![no_std]
#![no_main]

use panic_halt as _;

use core::fmt::Write;

use cortex_m_rt::entry;
use heapless::String;
use nb::block;
use stm32f1xx_hal::{pac, prelude::*, timer::Timer};
use stm32f1xx_hal::spi::{Mode, Phase, Polarity, Spi};
use stm32f1xx_hal::time;
use stm32f1xx_hal::i2c::{BlockingI2c, DutyCycle, Mode as I2cMode};

use embedded_graphics::{
    mono_font::{ascii::FONT_6X10, MonoTextStyleBuilder},
    pixelcolor::BinaryColor,
    prelude::*,
    text::{Baseline, Text},
};
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};
use max7219::MAX7219;


#[entry]
fn main() -> ! {
    // Get access to the core peripherals from the cortex-m crate
    let cp = cortex_m::Peripherals::take().unwrap();
    // Get access to the device specific peripherals from the peripheral access crate
    let dp = pac::Peripherals::take().unwrap();

    // Take ownership over the raw flash and rcc devices and convert them into the corresponding
    // HAL structs
    let mut flash = dp.FLASH.constrain();
    let rcc = dp.RCC.constrain();

    // Freeze the configuration of all the clocks in the system and store the frozen frequencies in
    // `clocks`
    let clocks = rcc.cfgr.freeze(&mut flash.acr);

    // Acquire the GPIO peripherals
    let mut gpiob = dp.GPIOB.split();
    let mut gpioc = dp.GPIOC.split();

    // Configure led
    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

	// Configure SPI
    let pins = (
        // clk
        gpiob.pb13.into_alternate_push_pull(&mut gpiob.crh),
        // miso
        gpiob.pb14.into_floating_input(&mut gpiob.crh),
        // mosi
        gpiob.pb15.into_alternate_push_pull(&mut gpiob.crh),
    );
    let load = gpiob.pb12.into_push_pull_output(&mut gpiob.crh);

    let spi_mode = Mode {
        polarity: Polarity::IdleLow,
        phase: Phase::CaptureOnFirstTransition,
    };
    let spi = Spi::spi2(dp.SPI2, pins, spi_mode, time::Hz(100_000), clocks);

	// Configure Max7219
    let mut max7219 = MAX7219::from_spi_cs(1, spi, load).unwrap();
    max7219.power_on().unwrap();
    max7219.set_intensity(0, 5).unwrap();

    // Configre I2C
    let scl = gpiob.pb10.into_alternate_open_drain(&mut gpiob.crh);
    let sda = gpiob.pb11.into_alternate_open_drain(&mut gpiob.crh);

    let i2c = BlockingI2c::i2c2(
        dp.I2C2,
        (scl, sda),
        I2cMode::Fast {
            frequency: time::Hz(400_000),
            duty_cycle: DutyCycle::Ratio2to1,
        },
        clocks,
        1000,
        10,
        1000,
        1000,
    );

    // Configure oled display
    let interface = I2CDisplayInterface::new(i2c);
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_buffered_graphics_mode();
    display.init().unwrap();

    let text_style = MonoTextStyleBuilder::new()
        .font(&FONT_6X10)
        .text_color(BinaryColor::On)
        .build();

    // Loop
    let mut counter = 0;

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        led.set_high();
        block!(timer.wait()).unwrap();
        led.set_low();

        let mut counter_str: String<8> = String::new();
        write!(counter_str, "{:08}", counter).unwrap();

        // Write the counter on the oled leds display
        display.clear();
        Text::with_baseline(counter_str.as_str(), Point::zero(), text_style, Baseline::Top)
            .draw(&mut display)
            .unwrap();
        display.flush().unwrap();

        // Write the counter on the 7 leds display
        let counter_bytes: [u8; 8] = counter_str.into_bytes()[..].try_into().unwrap();
        max7219.write_str(0, &counter_bytes, 0b00000000).unwrap();

        counter += 1;
    }
}

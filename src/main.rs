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

    // Acquire the GPIOC peripheral
    let mut gpioc = dp.GPIOC.split();

    // Configure gpio C pin 13 as a push-pull output. The `crh` register is passed to the function
    // in order to configure the port. For pins 0-7, crl should be passed instead.
    let mut led = gpioc.pc13.into_push_pull_output(&mut gpioc.crh);
    // Configure the syst timer to trigger an update every second
    let mut timer = Timer::syst(cp.SYST, &clocks).counter_hz();
    timer.start(1.Hz()).unwrap();

	// Configure SPI
    let mut gpiob = dp.GPIOB.split();

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

    let mut counter = 0;

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        led.set_high();
        block!(timer.wait()).unwrap();
        led.set_low();

        let mut counter_str: String<8> = String::new();
        write!(counter_str, "{:08}", counter).unwrap();
        let counter_bytes: [u8; 8] = counter_str.into_bytes()[..].try_into().unwrap();
        max7219.write_str(0, &counter_bytes, 0b00000000).unwrap();

        counter += 1;
    }
}

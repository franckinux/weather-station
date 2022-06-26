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

use bme280_multibus::{i2c::Address, Bme280, Sample};
// use embedded_graphics::{
//     mono_font::{ascii::FONT_8X13, MonoTextStyleBuilder},
//     pixelcolor::BinaryColor,
//     prelude::*,
//     text::{Baseline, Text},
// };
use max7219::MAX7219;
use shared_bus::BusManagerSimple;
use ssd1306::{prelude::*, I2CDisplayInterface, Ssd1306};


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
    let i2c_bus = BusManagerSimple::new(i2c);

    // Configure oled display
    let interface = I2CDisplayInterface::new(i2c_bus.acquire_i2c());
    let mut display = Ssd1306::new(interface, DisplaySize128x64, DisplayRotation::Rotate0)
        .into_terminal_mode();
    display.init().unwrap();

    // Configure BMP280
    const BME280_SETTINGS: bme280_multibus::Settings = bme280_multibus::Settings {
        config: bme280_multibus::Config::reset()
            .set_standby_time(bme280_multibus::Standby::Millis1000)
            .set_filter(bme280_multibus::Filter::X16),
        ctrl_meas: bme280_multibus::CtrlMeas::reset()
            .set_osrs_t(bme280_multibus::Oversampling::X8)
            .set_osrs_p(bme280_multibus::Oversampling::X8)
            .set_mode(bme280_multibus::Mode::Normal),
        ctrl_hum: bme280_multibus::Oversampling::X8,
    };

    let mut bme280 = Bme280::from_i2c(i2c_bus.acquire_i2c(), Address::SdoGnd).unwrap();
    bme280.settings(&BME280_SETTINGS).unwrap();

    // Loop
    let mut counter = 0;
    let mut sample: Sample;

    // Wait for the timer to trigger an update and change the state of the LED
    loop {
        block!(timer.wait()).unwrap();
        led.set_high();
        block!(timer.wait()).unwrap();
        led.set_low();

        // Write the temperature, pressure and humidity on the oled display
        // display.clear().unwrap();
        sample = bme280.sample().unwrap();

        let mut temp_str: String<8> = String::new();
        write!(temp_str, "{:.2}°C", sample.temperature).unwrap();
        display.set_position(0, 0).unwrap();
        display.write_str(temp_str.as_str()).unwrap();

        let mut pressure_str: String<16> = String::new();
        write!(pressure_str, "{:.1}hPa", sample.pressure / 100.0).unwrap();
        display.set_position(0, 1).unwrap();
        display.write_str(pressure_str.as_str()).unwrap();

        let mut humidity_str: String<8> = String::new();
        write!(humidity_str, "{:.2}%", sample.humidity).unwrap();
        display.set_position(0, 2).unwrap();
        display.write_str(humidity_str.as_str()).unwrap();

        // Write the counter on the 7 leds display
        let mut counter_str: String<8> = String::new();
        write!(counter_str, "{:08}", counter).unwrap();
        let counter_bytes: [u8; 8] = counter_str.into_bytes()[..].try_into().unwrap();
        max7219.write_str(0, &counter_bytes, 0b00000000).unwrap();

        counter += 1;
    }
}

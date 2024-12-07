#![no_main]
#![no_std]

use core::sync::atomic::{AtomicU16, Ordering};
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::{Async, Blocking};
use embassy_stm32::peripherals::{DMA1_CH5, PA10, TIM1};
use embassy_stm32::spi::{self, Mode, Spi};
use embassy_stm32::time::{mhz, Hertz};
use embassy_stm32::timer::simple_pwm::{PwmPin, SimplePwm};
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use i2cdriver::I2CDriver;
use static_cell::StaticCell;
use {defmt_rtt as _, panic_probe as _};

mod bmp280;
mod ens160;
mod i2cdriver;
mod ili9341;
mod spidriver;

static LCD_DUTY_CYCLE: AtomicU16 = AtomicU16::new(0);
static LCD_MAX_DUTY_CYCLE: AtomicU16 = AtomicU16::new(0);

// #[embassy_executor::task]
// async fn led_task(led: AnyPin) {
//     // Configure the LED pin as a push pull output and obtain handler.
//     // On the Nucleo F091RC there's an on-board LED connected to pin PA5.
//     let mut led: Output<'_> = Output::new(led, Level::Low, Speed::Low);

//     loop {
//         let del = BLINK_MS.load(Ordering::Relaxed);
//         info!("Value of del is {}", del);
//         Timer::after_millis(del.into()).await;
//         info!("LED toggling");
//         led.toggle();
//     }
// }

#[embassy_executor::task]
async fn pwm_task(
    pwm_pin: PA10,
    timer: TIM1,
    dma_channel: &'static mut Mutex<ThreadModeRawMutex, DMA1_CH5>,
) {
    let _dma = dma_channel.lock().await;
    let lcd_brightness = PwmPin::new_ch3(pwm_pin, embassy_stm32::gpio::OutputType::PushPull);
    let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        timer,
        None,
        None,
        Some(lcd_brightness),
        None,
        Hertz(1000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let max_duty = pwm.max_duty_cycle();
    defmt::info!("{}", max_duty);
    LCD_DUTY_CYCLE.store(max_duty, Ordering::Relaxed);
    LCD_MAX_DUTY_CYCLE.store(max_duty, Ordering::Relaxed);
    let pwm_channel = embassy_stm32::timer::Channel::Ch3;
    pwm.channel(pwm_channel).set_duty_cycle(max_duty / 5);
    pwm.channel(pwm_channel).enable();
    loop {
        let del = LCD_DUTY_CYCLE.load(Ordering::Relaxed);
        pwm.channel(pwm_channel).set_duty_cycle(del as u16);
        Timer::after_millis(1).await;
    }
}

bind_interrupts!(struct Irqs {
    I2C1 => i2c::EventInterruptHandler<peripherals::I2C1>, i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize and create handle for devicer peripherals
    let mut p = embassy_stm32::init(Default::default());

    // Configure the button pin and obtain handler.
    // On the Nucleo F091RC there is a button connected to pin PC13.

    // Create and initialize a delay variable to manage delay loop
    let mut del_var = 2000;

    // Spawn LED blinking task
    // spawner.spawn(led_task(p.PA5.degrade())).unwrap();

    // Blink duration value to global context

    static SHARED_I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async>>> =
        StaticCell::new();
    static SHARED_SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Blocking>>> =
        StaticCell::new();
    static SHARED_DC: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
    static SHARED_RST: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
    static SHARED_DMA1_CH5: StaticCell<Mutex<ThreadModeRawMutex, DMA1_CH5>> = StaticCell::new();

    let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up);

    let i2c_config = i2c::Config::default();
    let i2c1: i2c::I2c<'_, embassy_stm32::mode::Async> = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH2,
        p.DMA1_CH3,
        Hertz(100_000),
        i2c_config,
    );

    let mut spi_config = spi::Config::default();
    spi_config.bit_order = spi::BitOrder::LsbFirst;
    spi_config.mode = Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };
    spi_config.frequency = Hertz(16_000_000);

    let spi = Spi::new_blocking(
        p.SPI2,
        p.PB13, //SCK
        p.PB15, //MOSI
        p.PB14,  //MISO
        // p.DMA1_CH5,
        // p.DMA1_CH4,
        spi::Config::default(),
    );

    let cs = p.PA8;
    let dc = p.PA9;
    let rst = p.PA3;
    let mut cs = Output::new(cs, Level::High, Speed::VeryHigh);
    let dc = Output::new(dc, Level::High, Speed::VeryHigh);
    let mut rst = Output::new(rst, Level::High, Speed::VeryHigh);
    rst.set_low();

    let static_i2c = SHARED_I2C.init(Mutex::new(i2c1));
    let static_spi = SHARED_SPI.init(Mutex::new(spi));
    let static_dc = SHARED_DC.init(Mutex::new(dc));
    let static_rst = SHARED_RST.init(Mutex::new(rst));

    let spidriver = spidriver::SPIDriver::new(static_spi, static_dc, static_rst);

    let bmp280_driver = I2CDriver::new(static_i2c, 0x76);

    let ens160_driver = I2CDriver::new(static_i2c, 0x53);

    // spawner.spawn(ens160::ens_task(ens160_driver)).unwrap();

    // spawner.spawn(bmp280::bmp_task(bmp280_driver)).unwrap();

    spawner.spawn(ili9341::screen_task(spidriver)).unwrap();

    let static_dma1_ch5: &mut Mutex<ThreadModeRawMutex, DMA1_CH5> =
        SHARED_DMA1_CH5.init(Mutex::new(p.DMA1_CH5));
    spawner
        .spawn(pwm_task(p.PA10, p.TIM1, static_dma1_ch5))
        .unwrap();

    // Timer::after_millis(200).await;
    // rst.set_low();
    // Timer::after_millis(200).await;
    // rst.set_high();
    // Timer::after_millis(200).await;
    cs.set_low();

    loop {
        // defmt::info!("loop");
        // Check if button got pressed
        button.wait_for_rising_edge().await;
        Timer::after_millis(1).await;
        info!("rising_edge");
        del_var = del_var - 200;
        // // If updated delay value drops below 200 then reset it back to starting value
        if del_var < 200 {
            del_var = LCD_MAX_DUTY_CYCLE.load(Ordering::Relaxed);
        }
        // Updated delay value to global context
        LCD_DUTY_CYCLE.store(del_var, Ordering::Relaxed);
        let mut d = static_rst.lock().await;
        if d.is_set_high() {
            d.set_low();
        } else {
            d.set_high();
        }
    }
}

#[defmt::panic_handler]
fn panic() -> ! {
    defmt::info!("Panic");
    panic_probe::hard_fault();
}

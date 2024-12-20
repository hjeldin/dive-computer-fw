#![no_main]
#![no_std]

use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use defmt::info;
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::mode::Async;
use embassy_stm32::peripherals::{DMA1_CH1, DMA1_CH3};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, peripherals};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::StaticCell;
use crate::tasks::enter_button::btn_enter_task;
use crate::tasks::next_button::btn_right_task;
use {defmt_rtt as _, panic_probe as _};
use crate::i2cdriver::I2CDriver;
use crate::tasks::buzzer::buzzer_pwm_task;
use crate::tasks::lcd_brightness::lcd_brightness_task;
use crate::tasks::no_interaction::no_interaction_task;

mod bmp280;
mod decotask;
mod ens160;
mod i2cdriver;
mod ili9341;
mod ms5837;
mod spidriver;
mod state;
mod tasks;

static LCD_DUTY_CYCLE: AtomicU16 = AtomicU16::new(0);
static LCD_MAX_DUTY_CYCLE: AtomicU16 = AtomicU16::new(0);
static LCD_REFRESH: AtomicBool = AtomicBool::new(false);
static LCD_NEXT_ITEM: AtomicBool = AtomicBool::new(false);
static LCD_ENTER_ITEM: AtomicBool = AtomicBool::new(false);
static TRIGGER_BUZZ: AtomicU16 = AtomicU16::new(0);
static TRIGGER_VOLUME: AtomicU16 = AtomicU16::new(0);
static INTERACTION: AtomicBool = AtomicBool::new(false);
static LOW_POWER_MODE: AtomicBool = AtomicBool::new(false);

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
});

pub static STATE: Mutex<ThreadModeRawMutex, state::State> = Mutex::new(state::State {
    time: [0; 4],
    pressure: 0.0,
    temperature: 0.0,
});

#[embassy_executor::main]
async fn main(spawner: Spawner) {
    // Initialize and create handle for devicer peripherals
    let mut defaults = embassy_stm32::Config::default();
    // set the internal clock to run at max (80MHz)
    defaults.rcc.hsi = true;
    defaults.rcc.pll = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSI,
        mul: embassy_stm32::rcc::PllMul::MUL10,
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7),
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2),
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
    });
    defaults.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
    defaults.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
    defaults.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    defaults.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    let p = embassy_stm32::init(defaults);

    let mut del_var = 2000;

    static SHARED_I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async>>> =
        StaticCell::new();
    static SHARED_SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async>>> =
        StaticCell::new();
    static SHARED_DC: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
    static SHARED_RST: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
    static SHARED_DMA1_CH3: StaticCell<Mutex<ThreadModeRawMutex, DMA1_CH3>> = StaticCell::new();
    static SHARED_DMA1_CH1: StaticCell<Mutex<ThreadModeRawMutex, DMA1_CH1>> = StaticCell::new();

    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = embassy_time::Duration::from_millis(1000);

    let i2c1: i2c::I2c<'_, embassy_stm32::mode::Async> = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz(100_000),
        i2c_config,
    );

    let mut spi_config = spi::Config::default();
    spi_config.bit_order = spi::BitOrder::MsbFirst;
    spi_config.frequency = Hertz(4_000_000);

    let spi = Spi::new(
        p.SPI2, p.PB13, //SCK
        p.PB15, //MOSI
        p.PB14, //MISO
        p.DMA1_CH5, p.DMA1_CH4, spi_config,
    );

    let cs = p.PA8;
    let dc = p.PA9;
    let rst = p.PB4;
    let mut cs: Output<'_> = Output::new(cs, Level::High, Speed::VeryHigh);
    let dc = Output::new(dc, Level::High, Speed::VeryHigh);
    let mut rst = Output::new(rst, Level::High, Speed::VeryHigh);
    rst.set_low();

    let static_i2c = SHARED_I2C.init(Mutex::new(i2c1));

    let static_dc = SHARED_DC.init(Mutex::new(dc));
    let static_rst = SHARED_RST.init(Mutex::new(rst));

    // let spidriver = spidriver::SPIDriver::new(static_spi, static_dc, static_rst);

    let bmp280_driver = I2CDriver::new(static_i2c, 0x76);

    let ens160_driver = I2CDriver::new(static_i2c, 0x53);

    let ms5837_driver = I2CDriver::new(static_i2c, 0x76);


    let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up);

    let button_right = ExtiInput::new(p.PA1, p.EXTI1, Pull::Up);


    let button_enter = ExtiInput::new(p.PA4, p.EXTI4, Pull::Up);

    let static_dma1_ch3: &mut Mutex<ThreadModeRawMutex, DMA1_CH3> =
        SHARED_DMA1_CH3.init(Mutex::new(p.DMA1_CH3));
    let static_dma1_ch1: &mut Mutex<ThreadModeRawMutex, DMA1_CH1> =
        SHARED_DMA1_CH1.init(Mutex::new(p.DMA1_CH1));


    // spawner.spawn(ens160::ens_task(ens160_driver)).unwrap();
    // spawner.spawn(bmp280::bmp_task(bmp280_driver)).unwrap();
    // spawner.spawn(ms5837::ms5837_task(ms5837_driver)).unwrap();
    spawner
        .spawn(ili9341::screen_task(spi, static_dc, static_rst))
        .unwrap();
    spawner.spawn(decotask::deco_task()).unwrap();
    spawner.spawn(btn_right_task(button_right)).unwrap();
    spawner.spawn(btn_enter_task(button_enter)).unwrap();
    spawner
        .spawn(lcd_brightness_task(p.PA10, p.TIM1, static_dma1_ch1))
        .unwrap();
    spawner
        .spawn(buzzer_pwm_task(p.PB5, p.TIM3, static_dma1_ch3))
        .unwrap();
    spawner.spawn(no_interaction_task()).unwrap();

    cs.set_low();

    loop {
        // defmt::info!("loop");
        // Check if button got pressed
        button.wait_for_rising_edge().await;
        Timer::after_millis(100).await;
        if(LOW_POWER_MODE.load(Ordering::Relaxed) == true){
            info!("[MAIN] Low power mode");
            return;
        }
        del_var = del_var - 2000;
        info!("Set LCD brightness to {}", del_var);
        // // // If updated delay value drops below 200 then reset it back to starting value
        if del_var < 2000 {
            del_var = LCD_MAX_DUTY_CYCLE.load(Ordering::Relaxed);
        }
        // // Updated delay value to global context
        LCD_DUTY_CYCLE.store(del_var, Ordering::Relaxed);

        LCD_REFRESH.store(true, Ordering::Relaxed);

        // let mut d = static_rst.lock().await;
        // if d.is_set_high() {
        //     d.set_low();
        // } else {
        //     d.set_high();
        // }
    }
}
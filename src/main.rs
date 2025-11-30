#![no_main]
#![no_std]

use core::arch::asm;
use crate::i2cdriver::I2CDriver;
use crate::tasks::buzzer::buzzer_pwm_task;
use crate::tasks::enter_button::btn_enter_task;
use crate::tasks::lcd_brightness::lcd_brightness_task;
use crate::tasks::next_button::btn_right_task;
use crate::tasks::ninedof::ninedof_task;
use crate::tasks::no_interaction::no_interaction_task;
use core::mem;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use defmt::{info, unwrap, Debug2Format};
use embassy_executor::Spawner;
use embassy_stm32::exti::{self, ExtiInput};
use embassy_stm32::gpio::{Flex, Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
// use embassy_stm32::low_power::Executor;
use embassy_stm32::mode::Async;
use embassy_stm32::pac::ADC1;
use embassy_stm32::peripherals::{ADC1, DMA1_CH1, DMA1_CH3, DMA2_CH1, DMA2_CH4, PC1, PA6, PA7, PB0, PB1, PB10, PB11, QUADSPI, RCC, RTC, SDMMC1};
use embassy_stm32::rcc::{clocks, disable};
use embassy_stm32::rtc::{DateTime, DayOfWeek, Rtc, RtcConfig};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::spi::mode::Master as SpiMaster;
use embassy_stm32::i2c::mode::Master as I2cMaster;
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, pac, peripherals, sdmmc, usb, Peri, interrupt};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::rwlock::RwLock;
use embassy_time::Timer;
use static_cell::{StaticCell};
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::flash::{Flash, WRITE_SIZE};

use embassy_futures::join::join;
use embassy_stm32::adc::Adc;
use embassy_stm32::sdmmc::{DataBlock, Sdmmc};
use crate::tasks::batt_voltage_monitor::batt_voltage_monitor_task;
use crate::tasks::sd_card::sd_card_task;
// use crate::tasks::usart_hello::usart_hello;
use crate::tasks::usb_device::usb_device_task;
use crate::tasks::air_quality_request::air_quality_request_task;
use crate::tasks::flash_timestamp::flash_timestamp_task;
// use crate::tasks::usb_device::usb_device_task;
// use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterConfig};
// use embassy_usb_dfu::consts::DfuAttributes;
// use embassy_usb_dfu::{usb_dfu, Control, ResetImmediate};


mod bmp280;
mod decotask;
mod ens160;
mod i2cdriver;
mod ili9341;
mod mmc5983ma;
mod ms5837;
mod spidriver;
mod state;
mod tasks;
mod bluenrgm0a;

static LCD_MAX_DUTY_CYCLE: AtomicU16 = AtomicU16::new(0);
static LCD_REFRESH: AtomicBool = AtomicBool::new(false);
static LCD_NEXT_ITEM: AtomicBool = AtomicBool::new(false);
static LCD_ENTER_ITEM: AtomicBool = AtomicBool::new(false);
static TRIGGER_BUZZ: AtomicU16 = AtomicU16::new(0);
static TRIGGER_VOLUME: AtomicU16 = AtomicU16::new(0);
static INTERACTION: AtomicBool = AtomicBool::new(false);
static LOW_POWER_MODE: AtomicBool = AtomicBool::new(false);

bind_interrupts!(struct Irqs {
    EXTI1 => exti::InterruptHandler<interrupt::typelevel::EXTI1>;
    EXTI4 => exti::InterruptHandler<interrupt::typelevel::EXTI4>;
    EXTI15_10 => exti::InterruptHandler<interrupt::typelevel::EXTI15_10>;
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

pub static STATE: RwLock<ThreadModeRawMutex, state::State> = RwLock::new(state::State {
    time: [0; 4],
    pressure: 0.0,
    temperature: 0.0,
    gyro_x: 0.0,
    gyro_y: 0.0,
    gyro_z: 0.0,
    accel_x: 0.0,
    accel_y: 0.0,
    accel_z: 0.0,
    batt_voltage: 0.0,
    low_batt: false,
    power_good: false,
    charge_status: false,
    mag_x: 0.0,
    mag_y: 0.0,
    mag_z: 0.0,
    mag_heading: 0.0,
});

// #[cortex_m_rt::entry]
// fn main() -> ! {
//     Executor::take().run(|spawner| {
//         unwrap!(spawner.spawn(async_main(spawner)));
//     });
//
//     loop {
//         cortex_m::asm::wfi();
//     }
// }

static SHARED_I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async, I2cMaster>>> = StaticCell::new();
static SHARED_SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, SpiMaster>>> = StaticCell::new();
static SHARED_SPI_BLUENRG: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async, SpiMaster>>> = StaticCell::new();
static SHARED_DC: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
static SHARED_RST: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
static SHARED_NRG_RST: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
static SHARED_NRG_CS: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
static SHARED_DMA2_CH1: StaticCell<Mutex<ThreadModeRawMutex, Peri<DMA2_CH1>>> = StaticCell::new();
static SHARED_DMA1_CH1: StaticCell<Mutex<ThreadModeRawMutex, Peri<DMA1_CH1>>> = StaticCell::new();

static SHARED_SDMMC1: StaticCell<Mutex<ThreadModeRawMutex, Sdmmc<'static, SDMMC1>>> = StaticCell::new();

static LOW_BATT: StaticCell<Input<'static>> = StaticCell::new();
static POWER_GOOD: StaticCell<Input<'static>> = StaticCell::new();
static CHARGE_STATUS: StaticCell<Input<'static>> = StaticCell::new();
static ADC: StaticCell<Adc<ADC1>> = StaticCell::new();
static BAT_MON_EN: StaticCell<Output<'static>> = StaticCell::new();
static ADC_PIN: StaticCell<PC1> = StaticCell::new();

// static SERIAL: StaticCell<hal::uart::Uart0> = StaticCell::new();

// #[embassy_executor::task]
#[embassy_executor::main]
async fn async_main(spawner: Spawner) {
    info!("async_main");
    // Initialize and create handle for devicer peripherals
    let mut defaults = embassy_stm32::Config::default();
    // set the internal clock to run at max (80MHz)
    defaults.rcc.hsi = true;
    defaults.rcc.hse = Some(embassy_stm32::rcc::Hse {
        freq: Hertz(8_000_000),
        mode: embassy_stm32::rcc::HseMode::Oscillator,
    });
    defaults.rcc.pll = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSE,
        mul: embassy_stm32::rcc::PllMul::MUL20,
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7),
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2),
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
    });
    // // USB
    defaults.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSE,
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
        mul: embassy_stm32::rcc::PllMul::MUL12,
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7),
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2),
    });
    defaults.rcc.pllsai2 = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSE,
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
        mul: embassy_stm32::rcc::PllMul::MUL12,
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7),
        divq: None,
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2),
    });
    defaults.rcc.mux.clk48sel = embassy_stm32::rcc::mux::Clk48sel::PLLSAI1_Q;
    defaults.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
    defaults.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV1;
    defaults.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    defaults.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    defaults.rcc.mux.adcsel = embassy_stm32::rcc::mux::Adcsel::PLL1_Q;

    
    // defaults.rcc.hsi48 = Some(Hsi48Config { sync_from_usb: true }); // needed for USB

    // defaults.rcc.pll = Some(embassy_stm32::rcc::Pll {
    //     source: embassy_stm32::rcc::PllSource::HSI,
    //     mul: embassy_stm32::rcc::PllMul::MUL8,
    //     divp: Some(embassy_stm32::rcc::PllPDiv::DIV7),
    //     divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
    //     divr: Some(embassy_stm32::rcc::PllRDiv::DIV8),
    //     prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
    // });
    // defaults.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
    // defaults.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV4;
    // defaults.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    // defaults.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;

    // turn me on to enable debugging while asleep
    // defaults.enable_debug_during_sleep = false;
    let mut p = embassy_stm32::init(defaults);
    
    // Print clock information
    let clocks = clocks(&p.RCC);
    info!("=== Clock Configuration ===");
    if let Some(sys) = Option::<embassy_stm32::time::Hertz>::from(clocks.sys) {
        info!("  System Clock (SYSCLK): {} Hz", sys.0);
    }
    if let Some(hclk1) = Option::<embassy_stm32::time::Hertz>::from(clocks.hclk1) {
        info!("  AHB1 Clock (HCLK1): {} Hz", hclk1.0);
    }
    if let Some(hclk2) = Option::<embassy_stm32::time::Hertz>::from(clocks.hclk2) {
        info!("  AHB2 Clock (HCLK2): {} Hz", hclk2.0);
    }
    if let Some(hclk3) = Option::<embassy_stm32::time::Hertz>::from(clocks.hclk3) {
        info!("  AHB3 Clock (HCLK3): {} Hz", hclk3.0);
    }
    if let Some(pclk1) = Option::<embassy_stm32::time::Hertz>::from(clocks.pclk1) {
        info!("  APB1 Clock (PCLK1): {} Hz", pclk1.0);
    }
    if let Some(pclk1_tim) = Option::<embassy_stm32::time::Hertz>::from(clocks.pclk1_tim) {
        info!("  APB1 Timer Clock: {} Hz", pclk1_tim.0);
    }
    if let Some(pclk2) = Option::<embassy_stm32::time::Hertz>::from(clocks.pclk2) {
        info!("  APB2 Clock (PCLK2): {} Hz", pclk2.0);
    }
    if let Some(pclk2_tim) = Option::<embassy_stm32::time::Hertz>::from(clocks.pclk2_tim) {
        info!("  APB2 Timer Clock: {} Hz", pclk2_tim.0);
    }
    if let Some(pll1_p) = Option::<embassy_stm32::time::Hertz>::from(clocks.pll1_p) {
        info!("  PLL1 P Output: {} Hz", pll1_p.0);
    }
    if let Some(pll1_q) = Option::<embassy_stm32::time::Hertz>::from(clocks.pll1_q) {
        info!("  PLL1 Q Output: {} Hz", pll1_q.0);
    }
    if let Some(pllsai1_p) = Option::<embassy_stm32::time::Hertz>::from(clocks.pllsai1_p) {
        info!("  PLLSAI1 P Output: {} Hz", pllsai1_p.0);
    }
    if let Some(pllsai1_q) = Option::<embassy_stm32::time::Hertz>::from(clocks.pllsai1_q) {
        info!("  PLLSAI1 Q Output: {} Hz", pllsai1_q.0);
    }
    if let Some(pllsai2_p) = Option::<embassy_stm32::time::Hertz>::from(clocks.pllsai2_p) {
        info!("  PLLSAI2 P Output: {} Hz", pllsai2_p.0);
    }
    if let Some(hsi) = Option::<embassy_stm32::time::Hertz>::from(clocks.hsi) {
        info!("  HSI: {} Hz", hsi.0);
    }
    if let Some(hse) = Option::<embassy_stm32::time::Hertz>::from(clocks.hse) {
        info!("  HSE: {} Hz", hse.0);
    }
    if let Some(rtc) = Option::<embassy_stm32::time::Hertz>::from(clocks.rtc) {
        info!("  RTC Clock: {} Hz", rtc.0);
    }
    // let adcclk = p.RCC.ccipr.read().adcsel().bits(); 
    // info!("ADCCLK selection bits: {}", adcclk);
    info!("===========================");

    // // Initialize RTC for flash timestamp task
    // let (mut rtc, time_provider) = embassy_stm32::rtc::Rtc::new(p.RTC, RtcConfig::default());
    // let _ = rtc.set_datetime(DateTime::from(2024, 12, 22, DayOfWeek::Sunday, 21, 37, 0, 0).unwrap());
    // let time_provider: &'static RtcTimeProvider = RTC_TIME_PROVIDER.init(time_provider);

    // // embassy_stm32::low_power::stop_with_rtc(rtc);

    let mut del_var = 2000;

    let mut i2c_config = i2c::Config::default();
    i2c_config.frequency = Hertz(400_000);
    i2c_config.timeout = embassy_time::Duration::from_millis(1000);

    let mut i2c1: i2c::I2c<'_, embassy_stm32::mode::Async, I2cMaster> = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        i2c_config,
    );

    let mut spi_nrg_config = spi::Config::default();
    spi_nrg_config.bit_order = spi::BitOrder::MsbFirst;
    spi_nrg_config.frequency = Hertz(2_000_000);

    spi_nrg_config.mode = spi::Mode {
        polarity: spi::Polarity::IdleLow,
        phase: spi::Phase::CaptureOnFirstTransition,
    };

    let spi_nrg = Spi::new(
        p.SPI1,
        p.PA5, //SCK
        p.PB5, //MOSI
        p.PB4, //MISO
        p.DMA1_CH3, p.DMA1_CH2, spi_nrg_config,
    );

    let nrg_rst = Output::new(p.PA15, Level::Low, Speed::VeryHigh);

    let nrg_cs = Output::new(p.PC4, Level::High, Speed::VeryHigh);

    let static_spi1: &'static mut Mutex<ThreadModeRawMutex, Spi<'static, Async, SpiMaster>> =
        SHARED_SPI_BLUENRG.init(Mutex::new(spi_nrg));
    let static_nrg_rst: &'static mut Mutex<ThreadModeRawMutex, Output<'static>> =
        SHARED_NRG_RST.init(Mutex::new(nrg_rst));
    let static_nrg_cs: &'static mut Mutex<ThreadModeRawMutex, Output<'static>> =
        SHARED_NRG_CS.init(Mutex::new(nrg_cs));

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
    let rst = p.PB12;
    let mut cs: Output<'_> = Output::new(cs, Level::High, Speed::VeryHigh);
    let dc = Output::new(dc, Level::High, Speed::VeryHigh);
    let mut rst = Output::new(rst, Level::High, Speed::VeryHigh);

    let static_i2c = SHARED_I2C.init(Mutex::new(i2c1));
    //
    let static_dc = SHARED_DC.init(Mutex::new(dc));
    let static_rst = SHARED_RST.init(Mutex::new(rst));

    let ms5837_driver = I2CDriver::new(static_i2c, 0x76);

    let button_right = ExtiInput::new(p.PA1, p.EXTI1, Pull::Up, Irqs);

    let button_enter = ExtiInput::new(p.PA4, p.EXTI4, Pull::Up, Irqs);
    
    let brightness_button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up, Irqs);
    //
    let static_dma_buzzer: &mut Mutex<ThreadModeRawMutex, Peri<DMA2_CH1>> =
        SHARED_DMA2_CH1.init(Mutex::new(p.DMA2_CH1));
    let static_dma1_ch1: &mut Mutex<ThreadModeRawMutex, Peri<DMA1_CH1>> =
        SHARED_DMA1_CH1.init(Mutex::new(p.DMA1_CH1));


    let static_sdmmc1: &mut Mutex<ThreadModeRawMutex, Sdmmc<'static, SDMMC1>> =
        SHARED_SDMMC1.init(Mutex::new(Sdmmc::new_4bit(
            p.SDMMC1,
            Irqs,
            p.DMA2_CH4,
            p.PC12,
            p.PD2,
            p.PC8,
            p.PC9,
            p.PC10,
            p.PC11,
            Default::default(),
        )));
    // spawner.spawn(no_interaction_task()).unwrap();
    let usb = p.USB_OTG_FS;
    let pa11 = p.PA11;
    let pa12 = p.PA12;
    let flash = p.FLASH;


    let ninedof_i2c = I2CDriver::new(
        static_i2c, 
        0x6b
    );
    let threedof_i2c = I2CDriver::new(
        static_i2c,
        0x30
    );
    // spawner.spawn(ens160::ens_task(ens160_driver)).unwrap();
    // spawner.spawn(bmp280::bmp_task(bmp280_driver)).unwrap();
    // spawner.spawn(decotask::deco_task()).unwrap();
    // spawner
    // .spawn(bluenrgm0a::ble_peripheral_task(
    //     static_spi1,
    //     static_nrg_rst,
    //     static_nrg_cs,
    //     &STATE,
    // ).expect("Failed to spawn BLE peripheral task"));
    
    cs.set_low();
        
        
    spawner.spawn(ms5837::ms5837_task(ms5837_driver, &STATE).expect("Failed to spawn MS5837 task"));
    spawner
        .spawn(lcd_brightness_task(
            brightness_button,
            p.PA10,
            p.TIM1,
            static_dma1_ch1
        ).expect("Failed to spawn LCD brightness task"));
    spawner
        .spawn(ili9341::screen_task(spi, static_dc, static_rst, &STATE).expect("Failed to spawn ILI9341 task"));
    
    spawner.spawn(btn_right_task(button_right).expect("Failed to spawn Right button task"));
    spawner.spawn(btn_enter_task(button_enter).expect("Failed to spawn Enter button task"));
    spawner
    .spawn(buzzer_pwm_task(
        p.PC7,
        p.TIM3,
        static_dma_buzzer
    ).expect("Failed to spawn Buzzer PWM task"));
    
    spawner.spawn(sd_card_task(static_sdmmc1, &STATE).expect("Failed to spawn SD card task"));
    
    spawner.spawn(flash_timestamp_task(
        p.QUADSPI,
        p.PB1,  // IO0
        p.PB0,  // IO1
        p.PA7,  // IO2
        p.PA6,  // IO3
        p.PB10, // CLK
        p.PB11, // NSS (Chip Select)
        p.RTC,  // RTC peripheral
    ).expect("Failed to spawn QUADSPI flash timestamp task"));
    
    spawner.spawn(ninedof_task(ninedof_i2c, threedof_i2c, &STATE).expect("Failed to spawn Ninedof task"));
    
    
    spawner.spawn(batt_voltage_monitor_task(
        p.PC3,
        p.PC0,
        p.PC5,
        p.PC2,
        p.ADC1,
        p.PC1,
        &STATE,
        p.DMA2_CH3,
    ).expect("Failed to spawn Battery voltage monitor task"));
    
    // spawner.spawn(air_quality_request_task(
        //     p.USART1,
        //     p.PB6,
        //     p.PB7,
        //     p.DMA2_CH6,
        //     p.DMA2_CH7,
        //     0x53,
        // )).unwrap();
        
    // spawner.spawn(usb_device_task(usb, pa12, pa11, flash)).unwrap();
    loop {
        Timer::after_millis(5000).await;
    }
}


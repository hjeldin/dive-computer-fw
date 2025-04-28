#![no_main]
#![no_std]

use core::arch::asm;
use crate::i2cdriver::I2CDriver;
// use crate::tasks::buzzer::buzzer_pwm_task;
use crate::tasks::enter_button::btn_enter_task;
use crate::tasks::lcd_brightness::lcd_brightness_task;
use crate::tasks::next_button::btn_right_task;
use crate::tasks::ninedof::ninedof_task;
use crate::tasks::no_interaction::no_interaction_task;
use core::mem;
use core::sync::atomic::{AtomicBool, AtomicU16, Ordering};
use defmt::{info, unwrap, Debug2Format};
use embassy_executor::Spawner;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::i2c::{self, I2c};
use embassy_stm32::low_power::Executor;
use embassy_stm32::mode::Async;
use embassy_stm32::pac::ADC1;
use embassy_stm32::peripherals::{ADC1, DMA1_CH1, DMA1_CH3, DMA2_CH4, PC1, RCC, SDMMC1};
use embassy_stm32::rcc::disable;
use embassy_stm32::rtc::{DateTime, DayOfWeek, Rtc, RtcConfig};
use embassy_stm32::spi::{self, Spi};
use embassy_stm32::time::Hertz;
use embassy_stm32::{bind_interrupts, pac, peripherals, sdmmc, usb};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use static_cell::{StaticCell};
use tasks::threedof::threedof_task;
use {defmt_rtt as _, panic_probe as _};
use embassy_stm32::flash::{Flash, WRITE_SIZE};

use embassy_futures::join::join;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::sdmmc::{DataBlock, Sdmmc};
use crate::tasks::batt_voltage_monitor::batt_voltage_monitor_task;
use crate::tasks::sd_card::sd_card_task;
// use crate::tasks::usb_device::usb_device_task;
// use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterConfig};
// use embassy_usb_dfu::consts::DfuAttributes;
// use embassy_usb_dfu::{usb_dfu, Control, ResetImmediate};


mod bmp280;
mod decotask;
mod ens160;
mod i2cdriver;
mod ili9341;
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
static RTC: StaticCell<Rtc> = StaticCell::new();

bind_interrupts!(struct Irqs {
    I2C1_EV => i2c::EventInterruptHandler<peripherals::I2C1>;
    I2C1_ER => i2c::ErrorInterruptHandler<peripherals::I2C1>;
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
    SDMMC1 => sdmmc::InterruptHandler<peripherals::SDMMC1>;
});

pub static STATE: Mutex<ThreadModeRawMutex, state::State> = Mutex::new(state::State {
    time: [0; 4],
    pressure: 0.0,
    temperature: 0.0,
});

#[cortex_m_rt::entry]
fn main() -> ! {
    Executor::take().run(|spawner| {
        unwrap!(spawner.spawn(async_main(spawner)));
    });

    loop {
        cortex_m::asm::wfi();
    }
}

static SHARED_I2C: StaticCell<Mutex<ThreadModeRawMutex, I2c<'static, Async>>> = StaticCell::new();
static SHARED_SPI: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async>>> = StaticCell::new();
static SHARED_SPI_BLUENRG: StaticCell<Mutex<ThreadModeRawMutex, Spi<'static, Async>>> = StaticCell::new();
static SHARED_DC: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
static SHARED_RST: StaticCell<Mutex<ThreadModeRawMutex, Output<'static>>> = StaticCell::new();
static SHARED_DMA1_CH3: StaticCell<Mutex<ThreadModeRawMutex, DMA1_CH3>> = StaticCell::new();
static SHARED_DMA1_CH1: StaticCell<Mutex<ThreadModeRawMutex, DMA1_CH1>> = StaticCell::new();

static SHARED_SDMMC1: StaticCell<Mutex<ThreadModeRawMutex, Sdmmc<'static, SDMMC1, DMA2_CH4>>> = StaticCell::new();

static LOW_BATT: StaticCell<Input<'static>> = StaticCell::new();
static POWER_GOOD: StaticCell<Input<'static>> = StaticCell::new();
static CHARGE_STATUS: StaticCell<Input<'static>> = StaticCell::new();
static ADC: StaticCell<Adc<ADC1>> = StaticCell::new();
static BAT_MON_EN: StaticCell<Output<'static>> = StaticCell::new();
static ADC_PIN: StaticCell<PC1> = StaticCell::new();

#[embassy_executor::task]
async fn async_main(spawner: Spawner) {
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
        mul: embassy_stm32::rcc::PllMul::MUL10,
        divp: Some(embassy_stm32::rcc::PllPDiv::DIV7),
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
        divr: Some(embassy_stm32::rcc::PllRDiv::DIV2),
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
    });
    // USB
    defaults.rcc.pllsai1 = Some(embassy_stm32::rcc::Pll {
        source: embassy_stm32::rcc::PllSource::HSE,
        prediv: embassy_stm32::rcc::PllPreDiv::DIV1,
        mul: embassy_stm32::rcc::PllMul::MUL12,
        divp: None,
        divq: Some(embassy_stm32::rcc::PllQDiv::DIV2),
        divr: None,
    });
    defaults.rcc.mux.clk48sel = embassy_stm32::rcc::mux::Clk48sel::PLLSAI1_Q;
    defaults.rcc.sys = embassy_stm32::rcc::Sysclk::PLL1_R;
    defaults.rcc.ahb_pre = embassy_stm32::rcc::AHBPrescaler::DIV2;
    defaults.rcc.apb1_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    defaults.rcc.apb2_pre = embassy_stm32::rcc::APBPrescaler::DIV1;
    defaults.rcc.mux.adcsel = embassy_stm32::rcc::mux::Adcsel::SYS;

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

    // give ownership of rtc peripheral to the executor
    let mut rtc = embassy_stm32::rtc::Rtc::new(p.RTC, RtcConfig::default());
    let _ = rtc.set_datetime(DateTime::from(2024, 12, 22, DayOfWeek::Sunday, 21, 37, 0).unwrap());
    let mut rtc = RTC.init(rtc);

    // embassy_stm32::low_power::stop_with_rtc(rtc);

    let mut del_var = 2000;

    let mut i2c_config = i2c::Config::default();
    i2c_config.timeout = embassy_time::Duration::from_millis(1000);

    let mut i2c1: i2c::I2c<'_, embassy_stm32::mode::Async> = i2c::I2c::new(
        p.I2C1,
        p.PB8,
        p.PB9,
        Irqs,
        p.DMA1_CH6,
        p.DMA1_CH7,
        Hertz(400_000),
        i2c_config,
    );

    let mut spi_nrg_config = spi::Config::default();
    spi_nrg_config.bit_order = spi::BitOrder::MsbFirst;
    spi_nrg_config.frequency = Hertz(4_000_000);

    let spi_nrg = Spi::new(
        p.SPI1,
        p.PA5, //SCK
        p.PB5, //MOSI
        p.PB4, //MISO
        p.DMA1_CH3, p.DMA1_CH2, spi_nrg_config,
    );

    let mut nrg_rst = Output::new(p.PA15, Level::Low, Speed::VeryHigh);

    let static_spi1 = SHARED_SPI_BLUENRG.init(Mutex::new(spi_nrg));

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
    //
    // // disable onboard led
    // let mut onboard_led = Output::new(p.PA5, Level::High, Speed::VeryHigh);
    // onboard_led.set_low();
    //
    //
    // rst.set_low();
    //
    let static_i2c = SHARED_I2C.init(Mutex::new(i2c1));
    //
    let static_dc = SHARED_DC.init(Mutex::new(dc));
    let static_rst = SHARED_RST.init(Mutex::new(rst));
    //
    // // let spidriver = spidriver::SPIDriver::new(static_spi, static_dc, static_rst);
    //
    // // let bmp280_driver = I2CDriver::new(static_i2c, 0x76);
    // //
    // // let ens160_driver = I2CDriver::new(static_i2c, 0x53);
    // //
    let ms5837_driver = I2CDriver::new(static_i2c, 0x76);
    //
    //
    // let mut button = ExtiInput::new(p.PC13, p.EXTI13, Pull::Up);

    let button_right = ExtiInput::new(p.PA1, p.EXTI1, Pull::Up);

    let button_enter = ExtiInput::new(p.PA4, p.EXTI4, Pull::Up);
    //
    // let static_dma1_ch3: &mut Mutex<ThreadModeRawMutex, DMA1_CH3> =
    //     SHARED_DMA1_CH3.init(Mutex::new(p.DMA1_CH3));
    let static_dma1_ch1: &mut Mutex<ThreadModeRawMutex, DMA1_CH1> =
        SHARED_DMA1_CH1.init(Mutex::new(p.DMA1_CH1));


    let static_sdmmc1: &mut Mutex<ThreadModeRawMutex, Sdmmc<'static, SDMMC1, DMA2_CH4>> =
        SHARED_SDMMC1.init(Mutex::new(Sdmmc::new_1bit(
            p.SDMMC1,
            Irqs,
            p.DMA2_CH4,
            p.PC12,
            p.PD2,
            p.PC8,
            Default::default(),
        )));


    // spawner.spawn(ens160::ens_task(ens160_driver)).unwrap();
    // spawner.spawn(bmp280::bmp_task(bmp280_driver)).unwrap();
    spawner.spawn(ms5837::ms5837_task(ms5837_driver, &STATE)).unwrap();
    // spawner
    //     .spawn(ili9341::screen_task(spi, static_dc, static_rst))
    //     .unwrap();
    // // spawner.spawn(decotask::deco_task()).unwrap();
    // spawner.spawn(btn_right_task(button_right)).unwrap();
    // spawner.spawn(btn_enter_task(button_enter)).unwrap();
    // spawner
    //     .spawn(lcd_brightness_task(
    //         p.PC13,
    //         p.EXTI13,
    //         p.PA10,
    //         p.TIM1,
    //         static_dma1_ch1
    //     ))
    //     .unwrap();
    // spawner
    //     .spawn(buzzer_pwm_task(
    //         p.PC4, 
    //         p.TIM3, 
    //         static_dma1_ch3
    //     ))
    //     .unwrap();
    // spawner.spawn(no_interaction_task()).unwrap();

    // spawner.spawn(usb_device_task(p.FLASH, p.USB_OTG_FS, p.PA12, p.PA11, spawner)).unwrap();
    spawner.spawn(sd_card_task(static_sdmmc1, &STATE)).unwrap();

    let ninedof_i2c = I2CDriver::new(
        static_i2c, 0x6b
    );
    let threedof_i2c = I2CDriver::new(
        static_i2c,
        0x30
    );
    // spawner.spawn(ninedof_task(ninedof_i2c)).unwrap();
    //// spawner.spawn(threedof_task(threedof_i2c)).unwrap();

    cs.set_low();
    // get input pin PB10, PB11, PB12
    let low_batt: &'static mut _ = LOW_BATT.init(Input::new(p.PC3, Pull::Up));
    let power_good: &'static mut _ = POWER_GOOD.init(Input::new(p.PC0, Pull::Up));
    let charge_status: &'static mut _ = CHARGE_STATUS.init(Input::new(p.PC5, Pull::Up));
    let adc_bat_v: &'static mut _ = ADC.init(Adc::new(p.ADC1));
    adc_bat_v.set_sample_time(SampleTime::CYCLES640_5);
    let bat_mon_en: &'static mut _ = BAT_MON_EN.init(Output::new(p.PC2, Level::High, Speed::VeryHigh));
    let adc_pin: &'static mut _ = ADC_PIN.init(p.PC1);

    // spawner.spawn(batt_voltage_monitor_task(
    //     low_batt,
    //     power_good,
    //     charge_status,
    //     adc_bat_v,
    //     bat_mon_en,
    //     adc_pin
    // )).unwrap();

    loop {
        info!("main loop");
        Timer::after_millis(10000).await;
    }
}


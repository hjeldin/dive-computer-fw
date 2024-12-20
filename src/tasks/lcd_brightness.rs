use core::sync::atomic::Ordering;
use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::info;
use embassy_stm32::peripherals::{DMA1_CH1, PA10, TIM1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use crate::{INTERACTION, LCD_DUTY_CYCLE, LCD_MAX_DUTY_CYCLE, LOW_POWER_MODE};

#[embassy_executor::task]
pub async fn lcd_brightness_task(
    pwm_pin: PA10,
    timer: TIM1,
    dma_channel: &'static mut Mutex<ThreadModeRawMutex, DMA1_CH1>,
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
        Timer::after_millis(100).await;
        if(LOW_POWER_MODE.load(Ordering::Relaxed) == true){
            pwm.channel(pwm_channel).set_duty_cycle(0);
            pwm.disable(pwm_channel);
            info!("[LCDBrightness] Low power mode");
            return;
        }
    }
}
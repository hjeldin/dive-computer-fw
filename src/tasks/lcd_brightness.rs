use crate::{INTERACTION, LCD_MAX_DUTY_CYCLE, LOW_POWER_MODE};
use core::mem;
use core::sync::atomic::Ordering;
use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::gpio::Pull;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{DMA1_CH1, PA10, PC13, TIM1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn lcd_brightness_task(
    mut brightness_button: ExtiInput<'static>,
    pwm_pin: Peri<'static, PA10>,
    timer: Peri<'static, TIM1>,
    dma_channel: &'static mut Mutex<ThreadModeRawMutex, Peri<'static, DMA1_CH1>>,
) {
    let _dma = dma_channel.lock().await;
    let lcd_brightness = PwmPin::new(pwm_pin, embassy_stm32::gpio::OutputType::PushPull);
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
    let mut del_var = max_duty / 5;
    defmt::info!("Brightness max duty: {}", max_duty);
    LCD_MAX_DUTY_CYCLE.store(max_duty, Ordering::Relaxed);
    let pwm_channel = embassy_stm32::timer::Channel::Ch3;
    pwm.channel(pwm_channel).set_duty_cycle(max_duty / 12);
    pwm.channel(pwm_channel).enable();
    loop {
        brightness_button.wait_for_rising_edge().await;
        INTERACTION.store(true, Ordering::Relaxed);
        del_var -= max_duty / 5;
        if del_var < max_duty / 5 {
            del_var = LCD_MAX_DUTY_CYCLE.load(Ordering::Relaxed);
        }
        let del = del_var;
        pwm.channel(pwm_channel).set_duty_cycle(del as u16);
        Timer::after_millis(1000).await;
        if (LOW_POWER_MODE.load(Ordering::Relaxed) == true) {
            pwm.channel(pwm_channel).set_duty_cycle(0);
            pwm.disable(pwm_channel);
            info!("[LCDBrightness] Low power mode");
            break;
        }
    }

    mem::drop(pwm);
}

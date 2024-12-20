use core::sync::atomic::Ordering;
use embassy_stm32::peripherals::{DMA1_CH3, PB5, TIM3};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_time::Timer;
use crate::{TRIGGER_BUZZ, TRIGGER_VOLUME};

#[embassy_executor::task]
pub async fn buzzer_pwm_task(
    pwm_pin: PB5,
    timer: TIM3,
    dma_channel: &'static mut Mutex<ThreadModeRawMutex, DMA1_CH3>,
) {
    let _dma = dma_channel.lock().await;
    let lcd_brightness = PwmPin::new_ch2(pwm_pin, embassy_stm32::gpio::OutputType::PushPull);
    let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        timer,
        None,
        Some(lcd_brightness),
        None,
        None,
        Hertz(1000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let max_duty = pwm.max_duty_cycle();
    let pwm_channel = embassy_stm32::timer::Channel::Ch2;
    TRIGGER_VOLUME.store(max_duty/2, Ordering::Relaxed);
    pwm.set_frequency(Hertz(440));
    pwm.channel(pwm_channel).set_duty_cycle(TRIGGER_VOLUME.load(Ordering::Relaxed));
    pwm.channel(pwm_channel).enable();
    // pwm.channel(pwm_channel)
    Timer::after_millis(100).await;
    pwm.channel(pwm_channel).disable();
    loop {
        let buzz = TRIGGER_BUZZ.load(Ordering::Relaxed);
        if buzz == 0 {
            Timer::after_millis(100).await;
            continue;
        }
        pwm.set_frequency(Hertz(buzz as u32));
        pwm.channel(pwm_channel).enable();
        pwm.channel(pwm_channel).set_duty_cycle(TRIGGER_VOLUME.load(Ordering::Relaxed));
        defmt::info!("Buzzing with {}Hz at {}", buzz, TRIGGER_VOLUME.load(Ordering::Relaxed));
        Timer::after_millis(100).await;
        pwm.channel(pwm_channel).disable();
        TRIGGER_BUZZ.store(0, Ordering::Relaxed);
    }
}
use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;
use crate::{LCD_ENTER_ITEM, TRIGGER_BUZZ};

#[embassy_executor::task]
pub async fn btn_enter_task(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_rising_edge().await;
        LCD_ENTER_ITEM.store(true, Ordering::Relaxed);
        info!("Enter button pressed");
        TRIGGER_BUZZ.store(1000, Ordering::Relaxed);
        Timer::after_millis(100).await;
    }
}
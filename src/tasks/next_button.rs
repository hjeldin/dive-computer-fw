use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;
use crate::{LCD_NEXT_ITEM, TRIGGER_BUZZ};

#[embassy_executor::task]
pub async fn btn_right_task(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_rising_edge().await;
        LCD_NEXT_ITEM.store(true, Ordering::Relaxed);
        info!("Right button pressed");
        TRIGGER_BUZZ.store(100, Ordering::Relaxed);
        Timer::after_millis(100).await;
    }
}
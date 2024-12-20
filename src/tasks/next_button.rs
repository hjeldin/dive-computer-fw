use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;
use crate::{INTERACTION, LCD_NEXT_ITEM, LOW_POWER_MODE, TRIGGER_BUZZ};

#[embassy_executor::task]
pub async fn btn_right_task(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_rising_edge().await;
        INTERACTION.store(true, Ordering::Relaxed);
        LCD_NEXT_ITEM.store(true, Ordering::Relaxed);
        info!("Right button pressed");
        TRIGGER_BUZZ.store(100, Ordering::Relaxed);
        Timer::after_millis(100).await;

        if(LOW_POWER_MODE.load(Ordering::Relaxed) == true){
            info!("[NextButtonTask] Low power mode");
            return;
        }
    }
}
use crate::{INTERACTION, LCD_ENTER_ITEM, LCD_WAKE, LOW_POWER_MODE, TRIGGER_BUZZ};
use core::mem;
use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn btn_enter_task(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_rising_edge().await;
        
        // If in low power mode, wake up the system
        if LOW_POWER_MODE.load(Ordering::Relaxed) {
            info!("[EnterButtonTask] Button pressed - waking from low power mode");
            LOW_POWER_MODE.store(false, Ordering::Relaxed);
            INTERACTION.store(true, Ordering::Relaxed);
            LCD_ENTER_ITEM.store(true, Ordering::Relaxed);
            LCD_WAKE.signal(());
            TRIGGER_BUZZ.store(1000, Ordering::Relaxed);
        } else {
            // Normal operation
            INTERACTION.store(true, Ordering::Relaxed);
            LCD_ENTER_ITEM.store(true, Ordering::Relaxed);
            LCD_WAKE.signal(());
            info!("Enter button pressed");
            TRIGGER_BUZZ.store(1000, Ordering::Relaxed);
        }
        
        Timer::after_millis(100).await;
    }
}

use crate::{INTERACTION, LCD_NEXT_ITEM, LCD_WAKE, LOW_POWER_MODE, TRIGGER_BUZZ};
use core::mem;
use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn btn_right_task(mut input: ExtiInput<'static>) {
    loop {
        input.wait_for_rising_edge().await;
        crate::tasks::lcd_brightness::note_activity();
        
        // If in low power mode, wake up the system
        if LOW_POWER_MODE.load(Ordering::Relaxed) {
            info!("[NextButtonTask] Button pressed - waking from low power mode");
            LOW_POWER_MODE.store(false, Ordering::Relaxed);
            INTERACTION.store(true, Ordering::Relaxed);
            LCD_NEXT_ITEM.store(true, Ordering::Relaxed);
            LCD_WAKE.signal(());
            TRIGGER_BUZZ.signal(100);
        } else {
            // Normal operation
            INTERACTION.store(true, Ordering::Relaxed);
            LCD_NEXT_ITEM.store(true, Ordering::Relaxed);
            LCD_WAKE.signal(());
            info!("Right button pressed");
            TRIGGER_BUZZ.signal(100);
        }
        
        Timer::after_millis(100).await;
    }
}

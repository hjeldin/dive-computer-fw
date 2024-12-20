use core::sync::atomic::Ordering;
use defmt::info;
use embassy_stm32::exti::ExtiInput;
use embassy_time::{Instant, Timer};
use crate::{INTERACTION, LCD_NEXT_ITEM, LOW_POWER_MODE, TRIGGER_BUZZ};

#[embassy_executor::task]
pub async fn no_interaction_task() {
    let mut last_interaction = Instant::now();
    loop {
        let interaction = INTERACTION.load(Ordering::Relaxed);
        if(interaction) {
            INTERACTION.store(false, Ordering::Relaxed);
            last_interaction = Instant::now();
        }
        if(last_interaction.elapsed().as_millis() as u32) > 10_000 {
            defmt::info!("No interaction for 10 seconds, turn off all tasks");
            LOW_POWER_MODE.store(true, Ordering::Relaxed);
            return;
        }
        Timer::after_millis(1000).await;
    }
}
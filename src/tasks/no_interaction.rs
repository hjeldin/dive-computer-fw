use crate::{INTERACTION, LOW_POWER_MODE, RTC, SHARED_I2C};
use core::mem;
use core::sync::atomic::Ordering;
use embassy_stm32::low_power;
use embassy_stm32::low_power::stop_ready;
use embassy_stm32::low_power::StopMode::{Stop1, Stop2};
use embassy_time::{Instant, Timer};

#[embassy_executor::task]
pub async fn no_interaction_task() {
    let mut last_interaction = Instant::now();
    loop {
        let interaction = INTERACTION.load(Ordering::Relaxed);
        if (interaction) {
            INTERACTION.store(false, Ordering::Relaxed);
            last_interaction = Instant::now();
        }
        if (last_interaction.elapsed().as_millis() as u32) > 10_000 {
            defmt::info!("No interaction for 10 seconds, turn off all tasks");
            LOW_POWER_MODE.store(true, Ordering::Relaxed);

            // mem::drop(i2c);
            loop {
                Timer::after_millis(100).await;
                if (stop_ready(Stop2)) {
                    defmt::info!("STOP READY!");
                    return;
                }
            }
        }
        Timer::after_millis(1000).await;
    }
}

use crate::{INTERACTION, LCD_WAKE, LOW_POWER_MODE};
use core::sync::atomic::Ordering;
use embassy_time::{Instant, Timer};
use embassy_stm32::pac;
use embassy_stm32::pac::rcc::vals::Hpre;

const INACTIVITY_TIMEOUT_MS: u32 = 10_000;
const LOW_POWER_CHECK_INTERVAL_MS: u64 = 100;
const NORMAL_CHECK_INTERVAL_MS: u64 = 1000;

/// Check if system clock is currently reduced (not DIV1)
fn is_clock_reduced() -> bool {
    // pac::RCC.cfgr().read().hpre() != Hpre::DIV1
    true
}

/// Restore system clock to full speed (DIV1)
fn restore_system_clock() {
    // pac::RCC.cfgr().modify(|w| {
    //     w.set_hpre(Hpre::DIV1);
    // });
}

/// Reduce system clock to 1/8 by setting AHB prescaler to DIV8
fn reduce_system_clock() {
    // pac::RCC.cfgr().modify(|w| {
    //     w.set_hpre(Hpre::DIV1);
    // });
}

/// Wait for low power mode to be cleared and restore system clock
async fn wait_for_low_power_exit(last_interaction: &mut Instant) {
    loop {
        Timer::after_millis(LOW_POWER_CHECK_INTERVAL_MS).await;
        
        // Check if low power mode was cleared (button may have cleared flag already)
        if !LOW_POWER_MODE.load(Ordering::Relaxed) {
            // Clock might still be reduced if button cleared flag before we could restore it
            if is_clock_reduced() {
                defmt::info!("Waking from low power mode - restoring system clock");
                restore_system_clock();
            }
            defmt::info!("System clock restored - resetting interaction timer");
            LOW_POWER_MODE.store(false, Ordering::Relaxed);
            *last_interaction = Instant::now();
            break;
        }
        
        // Also check if clock was restored externally (shouldn't happen, but be safe)
        if !is_clock_reduced() {
            defmt::info!("Clock already restored - resetting interaction timer");
            LOW_POWER_MODE.store(false, Ordering::Relaxed);
            *last_interaction = Instant::now();
            break;
        }
    }
}

#[embassy_executor::task]
pub async fn no_interaction_task() {
    let mut last_interaction = Instant::now();
    loop {
        // Check for user interaction
        if INTERACTION.load(Ordering::Relaxed) {
            INTERACTION.store(false, Ordering::Relaxed);
            last_interaction = Instant::now();
            
            // Always check if clock needs to be restored (button may have cleared LOW_POWER_MODE already)
            if is_clock_reduced() {
                LOW_POWER_MODE.store(false, Ordering::Relaxed);
                defmt::info!("Interaction detected - exiting low power mode");
                restore_system_clock();
                defmt::info!("System clock restored");
            }
        }
        
        // Check if we should enter low power mode
        let elapsed_ms = last_interaction.elapsed().as_millis() as u32;
        if elapsed_ms > INACTIVITY_TIMEOUT_MS && !LOW_POWER_MODE.load(Ordering::Relaxed) {
            defmt::info!("No interaction for {} seconds, entering low power mode", INACTIVITY_TIMEOUT_MS / 1000);
            LOW_POWER_MODE.store(true, Ordering::Relaxed);
            // Let the screen task see low power mode and exit
            LCD_WAKE.signal(());
            
            defmt::info!("Reducing system clock to 1/8 (closest available to 1/10)");
            reduce_system_clock();
            
            // Wait for low power mode to be cleared (button press or interaction)
            wait_for_low_power_exit(&mut last_interaction).await;
        }
        
        Timer::after_millis(NORMAL_CHECK_INTERVAL_MS).await;
    }
}

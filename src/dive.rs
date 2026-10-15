//! Feeds sensor pressure into the deco model once a second and publishes
//! the result in `STATE` for the screen.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, info, warn};
use dive_computer_deco::tracker::{DivePhase, DiveTracker, Snapshot, TrackerConfig};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::rwlock::RwLock;
use embassy_time::{Duration, Instant, Ticker};

use crate::{state, LCD_WAKE};

/// True while a dive is in progress; low-power mode must not start then.
pub static DIVING: AtomicBool = AtomicBool::new(false);

/// A tick slower than this is logged as a warning.
const SLOW_TICK_US: u64 = 50_000;

#[embassy_executor::task]
pub async fn dive_task(state: &'static RwLock<ThreadModeRawMutex, state::State>) {
    let mut tracker = DiveTracker::new(tracker_config());
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut last = Instant::now();
    let mut phase = DivePhase::Calibrating;
    let mut last_sample = 0u32;
    #[cfg(feature = "sim-dive")]
    let mut diver = sim::SimDiver::new();

    loop {
        ticker.next().await;
        let now = Instant::now();
        let dt_s = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;

        // A reading the sensor task has not refreshed since the last tick is
        // stale: feed it as invalid, so the tracker withdraws the deco values
        #[cfg(not(feature = "sim-dive"))]
        let (pressure_bar, dt_s) = {
            let state = state.read().await;
            let fresh = state.pressure_samples != last_sample;
            last_sample = state.pressure_samples;
            (if fresh { state.pressure / 1000.0 } else { f32::NAN }, dt_s)
        };
        #[cfg(feature = "sim-dive")]
        let (pressure_bar, dt_s) = diver.step(dt_s * sim::SPEEDUP, &tracker.snapshot());

        let started = Instant::now();
        let snapshot = tracker.tick(pressure_bar, dt_s);
        let took_us = started.elapsed().as_micros();
        if took_us > SLOW_TICK_US {
            warn!("[Dive] Tick took {} us", took_us);
        } else {
            debug!("[Dive] Tick took {} us", took_us);
        }

        if snapshot.phase != phase {
            match snapshot.phase {
                DivePhase::Calibrating => {}
                DivePhase::Surface => info!("[Dive] At the surface, {} bar", tracker.surface_bar()),
                DivePhase::Diving => info!("[Dive] Dive started"),
            }
            phase = snapshot.phase;
        }
        let failed = snapshot.ceiling_m.is_none() || snapshot.ndl.is_none() || snapshot.plan.is_none();
        if snapshot.phase != DivePhase::Calibrating && failed {
            warn!("[Dive] Deco calculation failed at {} m", snapshot.depth_m);
        }

        DIVING.store(snapshot.phase == DivePhase::Diving, Ordering::Relaxed);
        publish(state, &snapshot).await;
        LCD_WAKE.signal(());
    }
}

fn tracker_config() -> TrackerConfig {
    let config = TrackerConfig::default();
    // Simulated time runs faster, so the rate window must cover several ticks
    #[cfg(feature = "sim-dive")]
    let config = TrackerConfig {
        rate_window_s: config.rate_window_s * sim::SPEEDUP,
        ..config
    };
    config
}

async fn publish(state: &RwLock<ThreadModeRawMutex, state::State>, snapshot: &Snapshot) {
    let mut state = state.write().await;
    state.depth_m = snapshot.depth_m;
    state.max_depth_m = snapshot.max_depth_m;
    state.dive_time_s = snapshot.dive_time_s;
    state.ascent_rate_m_min = snapshot.ascent_rate_m_min;
    state.diving = snapshot.phase == DivePhase::Diving;
    state.ceiling_m = snapshot.ceiling_m;
    state.guidance = snapshot.guidance;
    state.tts_min = snapshot.plan.map(|plan| plan.tts_min);
}

/// A diver who descends to 40 m, stays 20 minutes and then ascends,
/// honouring the stops the deco model asks for.
#[cfg(feature = "sim-dive")]
mod sim {
    use dive_computer_deco::tracker::Snapshot;

    /// Simulated seconds per real second.
    pub const SPEEDUP: f32 = 10.0;
    const SURFACE_BAR: f32 = 1.0;
    const METRES_PER_BAR: f32 = 10.197;
    const BOTTOM_M: f32 = 40.0;
    const DESCENT_M_PER_S: f32 = 18.0 / 60.0;
    const ASCENT_M_PER_S: f32 = 9.0 / 60.0;
    /// At the surface first, so calibration (10 ticks of `SPEEDUP` seconds)
    /// finishes before the descent.
    const SURFACE_WAIT_S: f32 = 150.0;
    /// Time after leaving the surface at which the ascent starts.
    const ASCENT_AT_S: f32 = 22.0 * 60.0;

    pub struct SimDiver {
        time_s: f32,
        depth_m: f32,
    }

    impl SimDiver {
        pub fn new() -> Self {
            Self { time_s: 0.0, depth_m: 0.0 }
        }

        /// Advance by `dt_s` simulated seconds; returns the pressure and the
        /// interval to feed the tracker.
        pub fn step(&mut self, dt_s: f32, snapshot: &Snapshot) -> (f32, f32) {
            self.time_s += dt_s;
            let dive_s = self.time_s - SURFACE_WAIT_S;
            if dive_s > 0.0 && dive_s < ASCENT_AT_S {
                self.depth_m = (self.depth_m + DESCENT_M_PER_S * dt_s).min(BOTTOM_M);
            } else if dive_s >= ASCENT_AT_S {
                // Never above the next stop
                let floor = snapshot.plan.and_then(|plan| plan.first_stop).map_or(0.0, |stop| stop.depth_m);
                self.depth_m = (self.depth_m - ASCENT_M_PER_S * dt_s).max(floor).max(0.0);
            }
            (SURFACE_BAR + self.depth_m / METRES_PER_BAR, dt_s)
        }
    }
}

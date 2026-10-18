//! Feeds sensor pressure into the deco model once a second and publishes
//! the result in `STATE` for the screen.

use core::sync::atomic::{AtomicBool, Ordering};

use defmt::{debug, info, warn};
use dive_computer_deco::tracker::{DivePhase, DiveTracker, Snapshot, TrackerConfig};
use embassy_sync::blocking_mutex::raw::{CriticalSectionRawMutex, ThreadModeRawMutex};
use embassy_sync::rwlock::RwLock;
use embassy_sync::signal::Signal;
use embassy_sync::watch::Watch;
use embassy_time::{Duration, Instant, Ticker};

use crate::{state, LCD_WAKE};

/// True while a dive is in progress; low-power mode must not start then.
pub static DIVING: AtomicBool = AtomicBool::new(false);

/// True while sensors should sample at full rate: the dive scene is open, the
/// tracker is diving, or the pressure sensor is deeper than the start depth.
/// At the surface only the pressure sensor runs, at `PRESSURE_IDLE_INTERVAL`.
/// One receiver each for the pressure, IMU and SD card tasks.
pub static HIGH_RATE: Watch<CriticalSectionRawMutex, bool, 3> = Watch::new_with(false);

/// Pressure polling interval at the surface, slow enough to save power and
/// fast enough to notice a submersion.
pub const PRESSURE_IDLE_INTERVAL: Duration = Duration::from_secs(5);

/// Gradient factors chosen in the settings scene, in percent: `(gf_low, gf_high)`.
pub static GRADIENT_FACTORS: Signal<CriticalSectionRawMutex, (u8, u8)> = Signal::new();

/// The tracker's gradient factors at boot, in percent: `(gf_low, gf_high)`.
pub fn default_gradient_factors() -> (u8, u8) {
    // The tracker starts from the deco defaults; the surface pressure does not matter here
    let config = dive_computer_deco::live::DecoConfig::new(1.0);
    (libm::roundf(config.gf_low * 100.0) as u8, libm::roundf(config.gf_high * 100.0) as u8)
}

static DIVE_SCENE_OPEN: AtomicBool = AtomicBool::new(false);
static SUBMERGED: AtomicBool = AtomicBool::new(false);

/// The screen task reports whether the dive scene is shown.
pub fn set_dive_scene_open(open: bool) {
    DIVE_SCENE_OPEN.store(open, Ordering::Relaxed);
    update_high_rate();
}

fn update_high_rate() {
    let on = DIVE_SCENE_OPEN.load(Ordering::Relaxed) || SUBMERGED.load(Ordering::Relaxed);
    HIGH_RATE.sender().send_if_modified(|current| {
        if *current == Some(on) {
            return false;
        }
        info!("[Dive] High-rate sampling {}", if on { "on" } else { "off" });
        if on {
            // Turns the screen back on
            crate::tasks::lcd_brightness::note_activity();
        }
        *current = Some(on);
        true
    });
}

/// A tick slower than this is logged as a warning.
const SLOW_TICK_US: u64 = 50_000;

#[embassy_executor::task]
pub async fn dive_task(state: &'static RwLock<ThreadModeRawMutex, state::State>) {
    let start_depth_m = tracker_config().start_depth_m;
    let mut tracker = DiveTracker::new(tracker_config());
    let mut ticker = Ticker::every(Duration::from_secs(1));
    let mut last = Instant::now();
    let mut phase = DivePhase::Calibrating;
    #[cfg(not(feature = "sim-dive"))]
    let mut last_sample = 0u32;
    #[cfg(not(feature = "sim-dive"))]
    let mut last_sample_at = Instant::now();
    #[cfg(feature = "sim-dive")]
    let mut diver = sim::SimDiver::new();

    loop {
        ticker.next().await;
        if let Some((low, high)) = GRADIENT_FACTORS.try_take() {
            info!("[Dive] Gradient factors {}/{}", low, high);
            tracker.set_gradient_factors(low as f32 / 100.0, high as f32 / 100.0);
        }
        let now = Instant::now();
        let dt_s = (now - last).as_micros() as f32 / 1_000_000.0;
        last = now;

        // A reading the sensor task has not refreshed since the last tick is
        // stale: feed it as invalid, so the tracker withdraws the deco values.
        // At the surface the sensor answers only every few seconds, so a
        // reading stays usable for one idle interval.
        #[cfg(not(feature = "sim-dive"))]
        let (pressure_bar, dt_s) = {
            let state = state.read().await;
            let fresh = state.pressure_samples != last_sample;
            last_sample = state.pressure_samples;
            if fresh {
                last_sample_at = now;
            }
            let high_rate = HIGH_RATE.try_get().unwrap_or(false);
            let usable = fresh || (!high_rate && now - last_sample_at <= PRESSURE_IDLE_INTERVAL + Duration::from_secs(1));
            (if usable { state.pressure / 1000.0 } else { f32::NAN }, dt_s)
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

        let diving = snapshot.phase == DivePhase::Diving;
        DIVING.store(diving, Ordering::Relaxed);
        // Full rate from the first reading below the start depth, so the
        // descent is recorded before the tracker confirms the dive
        SUBMERGED.store(diving || snapshot.depth_m > start_depth_m, Ordering::Relaxed);
        update_high_rate();
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

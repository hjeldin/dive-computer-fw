use crate::{INTERACTION, LCD_MAX_DUTY_CYCLE, LCD_WAKE, LOW_POWER_MODE};
use core::sync::atomic::{AtomicBool, AtomicU16, AtomicU8, Ordering};
use cortex_m::prelude::_embedded_hal_Pwm;
use defmt::info;
use embassy_futures::select::{Either4, select4};
use embassy_stm32::exti::ExtiInput;
use embassy_stm32::low_power::DeviceBusy;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{DMA1_CH1, PA10, TIM1};
use embassy_stm32::time::Hertz;
use embassy_stm32::timer::simple_pwm::PwmPin;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer, with_timeout};

/// The screen dims after this long without a button press or movement.
const DIM_AFTER_MS: u64 = 10_000;
/// Dimmed backlight, in percent of full. The lowest level at which the screen
/// is still readable depends on the panel: tune it on the device.
const DIM_DUTY_PERCENT: u32 = 2;
/// How often the dim conditions are checked, so a USB connection or a dive
/// start restores the brightness within this time.
const CHECK_INTERVAL_MS: u64 = 250;
/// The same while the screen is off. A USB connection and a dive start are
/// reported through `note_activity`, so this is only a fallback.
const OFF_CHECK_INTERVAL_MS: u64 = 60_000;
/// Longest wait for the panel to wake before the backlight goes on anyway.
const PANEL_WAKE_TIMEOUT_MS: u64 = 500;

/// Signalled on any use of the device other than the brightness button:
/// the other buttons, or the device being moved.
static ACTIVITY: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Backlight and screen-off time from the settings, see `configure`.
static BACKLIGHT_PERCENT: AtomicU8 = AtomicU8::new(stm_graphics::Settings::new().backlight_percent);
static SCREEN_OFF_S: AtomicU16 = AtomicU16::new(stm_graphics::Settings::new().screen_off_s);
static CONFIGURED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

/// Whether the panel should show a picture. Written by this task, followed
/// by the screen task, which owns the panel.
static PANEL_ON: AtomicBool = AtomicBool::new(true);
/// Signalled by the screen task once the panel shows a picture again.
static PANEL_AWAKE: Signal<CriticalSectionRawMutex, ()> = Signal::new();
/// Set when the brightness button asks for the next backlight level; the
/// screen task changes the setting, so the settings scene shows it and it is saved.
static BRIGHTNESS_STEP: AtomicBool = AtomicBool::new(false);

/// Report that the device is being used: restores a dimmed or blank screen.
pub fn note_activity() {
    ACTIVITY.signal(());
}

/// Set the backlight in percent and the idle time in seconds before the
/// screen turns off (0 keeps it on), from the settings.
pub fn configure(backlight_percent: u8, screen_off_s: u16) {
    BACKLIGHT_PERCENT.store(backlight_percent, Ordering::Relaxed);
    SCREEN_OFF_S.store(screen_off_s, Ordering::Relaxed);
    CONFIGURED.signal(());
}

/// Whether the screen task should keep the panel on.
pub fn panel_on() -> bool {
    PANEL_ON.load(Ordering::Relaxed)
}

/// Called by the screen task once the panel is on again after `panel_on`
/// turned true.
pub fn panel_awake() {
    PANEL_AWAKE.signal(());
}

/// Whether the brightness button asked for the next backlight level since
/// the last call.
pub fn take_brightness_step() -> bool {
    BRIGHTNESS_STEP.swap(false, Ordering::Relaxed)
}

/// What the screen shows.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
enum Screen {
    /// At the backlight setting.
    Bright,
    Dimmed,
    /// Backlight off and the panel asleep.
    Off,
}

#[embassy_executor::task]
pub async fn lcd_brightness_task(
    mut brightness_button: ExtiInput<'static>,
    pwm_pin: Peri<'static, PA10>,
    timer: Peri<'static, TIM1>,
    dma_channel: &'static mut Mutex<ThreadModeRawMutex, Peri<'static, DMA1_CH1>>,
) {
    let _dma = dma_channel.lock().await;
    let lcd_brightness = PwmPin::new(pwm_pin, embassy_stm32::gpio::OutputType::PushPull);
    let mut pwm = embassy_stm32::timer::simple_pwm::SimplePwm::new(
        timer,
        None,
        None,
        Some(lcd_brightness),
        None,
        Hertz(1000),
        embassy_stm32::timer::low_level::CountingMode::EdgeAlignedUp,
    );
    let max_duty = pwm.max_duty_cycle();
    defmt::info!("Brightness max duty: {}", max_duty);
    LCD_MAX_DUTY_CYCLE.store(max_duty, Ordering::Relaxed);
    let pwm_channel = embassy_stm32::timer::Channel::Ch3;
    let dim_duty = (max_duty / 100 * DIM_DUTY_PERCENT).max(1);
    let bright_duty = || (max_duty * BACKLIGHT_PERCENT.load(Ordering::Relaxed) as u32 / 100).max(1);
    pwm.channel(pwm_channel).set_duty_cycle(bright_duty());
    pwm.channel(pwm_channel).enable();
    // STOP mode stops the timer and freezes the PWM output, so the chip stays
    // out of it while the backlight is lit
    let mut keep_awake = Some(DeviceBusy::new_stop1());
    let mut last_activity = Instant::now();
    let mut screen = Screen::Bright;
    loop {
        let interval = if screen == Screen::Off { OFF_CHECK_INTERVAL_MS } else { CHECK_INTERVAL_MS };
        let event = select4(
            brightness_button.wait_for_rising_edge(),
            ACTIVITY.wait(),
            CONFIGURED.wait(),
            Timer::after_millis(interval),
        )
        .await;
        match event {
            Either4::First(()) => {
                INTERACTION.store(true, Ordering::Relaxed);
                last_activity = Instant::now();
                // A press on a dimmed or blank screen only restores it
                if screen == Screen::Bright {
                    BRIGHTNESS_STEP.store(true, Ordering::Relaxed);
                    LCD_WAKE.signal(());
                }
            }
            Either4::Second(()) => last_activity = Instant::now(),
            Either4::Third(()) => {
                if screen == Screen::Bright {
                    pwm.channel(pwm_channel).set_duty_cycle(bright_duty());
                }
            }
            Either4::Fourth(()) => {}
        }

        if LOW_POWER_MODE.load(Ordering::Relaxed) {
            pwm.channel(pwm_channel).set_duty_cycle(0);
            pwm.disable(pwm_channel);
            info!("[LCDBrightness] Low power mode - waiting for button press");
            brightness_button.wait_for_rising_edge().await;
            info!("[LCDBrightness] Button pressed - waking from low power mode");
            LOW_POWER_MODE.store(false, Ordering::Relaxed);
            INTERACTION.store(true, Ordering::Relaxed);
            pwm.channel(pwm_channel).set_duty_cycle(bright_duty());
            pwm.channel(pwm_channel).enable();
            last_activity = Instant::now();
            screen = Screen::Bright;
            continue;
        }

        let target = target_screen(last_activity.elapsed());
        if target == screen {
            continue;
        }
        info!("[LCDBrightness] Screen {}", target);
        if target == Screen::Off {
            // Backlight first, so the panel going dark is not seen
            pwm.channel(pwm_channel).set_duty_cycle(0);
            PANEL_ON.store(false, Ordering::Relaxed);
            LCD_WAKE.signal(());
            keep_awake = None;
        } else if screen == Screen::Off {
            keep_awake = Some(DeviceBusy::new_stop1());
            // Backlight after the panel is awake, so its start-up is not seen
            PANEL_AWAKE.reset();
            PANEL_ON.store(true, Ordering::Relaxed);
            LCD_WAKE.signal(());
            if with_timeout(Duration::from_millis(PANEL_WAKE_TIMEOUT_MS), PANEL_AWAKE.wait()).await.is_err() {
                defmt::warn!("[LCDBrightness] Panel did not report waking up");
            }
        }
        match target {
            Screen::Bright => pwm.channel(pwm_channel).set_duty_cycle(bright_duty()),
            Screen::Dimmed => pwm.channel(pwm_channel).set_duty_cycle(dim_duty),
            Screen::Off => {}
        }
        screen = target;
    }
}

/// What the screen should show after `idle` without use.
fn target_screen(idle: Duration) -> Screen {
    if !may_dim() {
        return Screen::Bright;
    }
    let off_after_s = SCREEN_OFF_S.load(Ordering::Relaxed);
    if off_after_s != 0 && idle >= Duration::from_secs(off_after_s as u64) {
        Screen::Off
    } else if idle >= Duration::from_millis(DIM_AFTER_MS) {
        Screen::Dimmed
    } else {
        Screen::Bright
    }
}

/// The screen may dim or turn off only at the surface with no computer attached.
fn may_dim() -> bool {
    let dive = crate::dive::HIGH_RATE.try_get().unwrap_or(false);
    !dive && !crate::tasks::usb_device::usb_connected()
}

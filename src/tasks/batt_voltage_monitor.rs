use defmt::info;
use defmt::error;
use embassy_stm32::adc::{Adc, SampleTime, AdcConfig, Resolution, RegularConversionMode, Averaging, vals::OversamplingShift, vals::OversamplingRatio};
use embassy_stm32::adc::AdcChannel;
use embassy_stm32::gpio::Flex;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{ADC1, PC0, PC1, PC2, PC3, PC5, DMA2_CH3};
use embassy_time::Timer;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::rwlock::RwLock;
use crate::state;
use crate::{LCD_WAKE, LOW_POWER_MODE};
use core::sync::atomic::Ordering;
use libm::floorf;

#[embassy_executor::task]
pub async fn batt_voltage_monitor_task(
    low_batt_pin: Peri<'static, PC3>,
    power_good_pin: Peri<'static, PC0>,
    charge_status_pin: Peri<'static, PC5>,
    bat_mon_en_pin: Peri<'static, PC2>,
    adc1: Peri<'static, ADC1>,
    mut adc_pin: Peri<'static, PC1>,
    state: &'static RwLock<ThreadModeRawMutex, state::State>,
    dma1_ch1: Peri<'static, DMA2_CH3>,
) {
    let low_batt = Input::new(low_batt_pin, Pull::Up);
    let power_good = Input::new(power_good_pin, Pull::Up);
    let charge_status = Input::new(charge_status_pin, Pull::Up);
    
    let mut config = AdcConfig::default();
    config.resolution = Some(Resolution::BITS12);
    config.averaging = Some(Averaging::Samples256);
    config.oversampling_shift = Some(OversamplingShift::SHIFT8);
    config.oversampling_ratio = Some(OversamplingRatio::RATIO8);
    let mut adc_bat_v = Adc::new_with_config(adc1, config);
    let mut bat_mon_en = Output::new(bat_mon_en_pin, Level::Low, Speed::VeryHigh);

    let mut vrefint_channel = adc_bat_v.enable_vrefint();
    Timer::after_millis(500).await;

    // Truncate to 2 decimal places
    let clamp_to_2_decimals = |value: f32| -> f32 {
        floorf(value * 100.0) / 100.0
    };

    const FILTER_ALPHA: f32 = 0.2;
    let mut filtered_voltage_mv: Option<f32> = None;

    // Read factory calibration value (only needs to be read once)
    let vrefint_cal = crate::pac::VREFINTCAL.data().read();
    // VREFINT calibration voltage for STM32L476 is 3.0V (3000mV)
    const VREFINT_CALIB_MV: f32 = 3000.0;
    // ADC resolution is 12-bit, so max count is 4095
    const ADC_MAX_COUNT: f32 = 4095.0;

    loop {
        // Check for low power mode
        if LOW_POWER_MODE.load(Ordering::Relaxed) {
            info!("[BattVoltageMonitor] Low power mode - disabling ADC");
            bat_mon_en.set_low();
            // Wait for low power mode to be cleared
            loop {
                Timer::after_millis(100).await;
                if !LOW_POWER_MODE.load(Ordering::Relaxed) {
                    info!("[BattVoltageMonitor] Waking from low power mode");
                    break;
                }
            }
            continue;
        }

        // Read VREFINT to calculate actual VDDA
        // This compensates for supply voltage variations
        let vrefint = adc_bat_v.blocking_read(&mut vrefint_channel, SampleTime::CYCLES2_5);
        
        // Calculate actual VDDA using: VDDA = VREFINT_CALIB_MV × (VREFINT_CAL / VREFINT_DATA)
        let vdda_actual_mv = VREFINT_CALIB_MV * (vrefint_cal as f32 / vrefint as f32);

        bat_mon_en.set_high();
        Timer::after_millis(500).await;
        let measured = adc_bat_v.blocking_read(&mut adc_pin, SampleTime::CYCLES2_5);

        // Convert ADC reading to voltage using calibrated VDDA
        let v_adc_mv: f32 = (measured as f32 / ADC_MAX_COUNT) * vdda_actual_mv;
        
        bat_mon_en.set_low();

        let filtered_voltage = match filtered_voltage_mv {
            Some(prev) => FILTER_ALPHA * v_adc_mv + (1.0 - FILTER_ALPHA) * prev,
            None => v_adc_mv, // First reading, no filtering
        };
        filtered_voltage_mv = Some(filtered_voltage);

        // R_top (VBAT -> ADC): 100k, R_bottom (ADC -> GND): 330k
        // v_adc = vs * 330 / (100 + 330), so vs = v_adc * (100 + 330) / 330
        let vs_mv = filtered_voltage * (100.0 + 330.0) / 330.0;

        // state.batt_voltage is in volts
        let clamped_voltage = clamp_to_2_decimals(vs_mv / 1000.0);

        {
            let mut state = state.write().await;
            state.batt_voltage = clamped_voltage;
            
            state.power_good = power_good.is_low();
            state.charge_status = charge_status.is_low();
            state.low_batt = low_batt.is_low();
        }
        LCD_WAKE.signal(());
        Timer::after_secs(1).await;
    }
}

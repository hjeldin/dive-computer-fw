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
use libm::floor;

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
    // let mut degraded_vrefint_channel = vrefint_channel.degrade_adc();
    // let mut degraded_adc_pin = adc_pin.degrade_adc();
    let mut degraded_vrefint_channel = adc_bat_v.enable_vrefint();
    Timer::after_millis(500).await;

    // Truncate to 2 decimal places
    let clamp_to_2_decimals = |value: f32| -> f32 {
        floor(value as f64 * 100.0) as f32 / 100.0
    };

    const FILTER_ALPHA: f32 = 0.2;
    let mut filtered_voltage_mv: Option<f32> = None;

    const DMA_BUF_LEN: usize = 512;
    let mut adc_dma_buf = [0u16; DMA_BUF_LEN];
    let mut measurements = [0u16; DMA_BUF_LEN / 2];
    // let mut ring_buffered_adc = adc_bat_v.into_ring_buffered(
    //     dma1_ch1,
    //     &mut adc_dma_buf,
    //     [(adc_bat_v, SampleTime::CYCLES640_5), (degraded_vrefint_channel, SampleTime::CYCLES640_5)].into_iter(),
    //     RegularConversionMode::Continuous,
    // );

    loop {

        bat_mon_en.set_high();       
        Timer::after_millis(50).await;
        let measured = adc_bat_v.blocking_read(&mut adc_pin, SampleTime::CYCLES2_5);

        let v_adc_mv:f32 = measured as f32 * 3.3 / 4095.0;
        
        bat_mon_en.set_low();
        Timer::after_millis(1000).await;

        let filtered_voltage = match filtered_voltage_mv {
            Some(prev) => FILTER_ALPHA * v_adc_mv + (1.0 - FILTER_ALPHA) * prev,
            None => v_adc_mv, // First reading, no filtering
        };
        filtered_voltage_mv = Some(filtered_voltage);

        // R1: 330k, R2: 100k so the voltage divider is vs * 330 / (330 + 100)
        let vs = v_adc_mv * (330.0 + 100.0) / 330.0;

        let clamped_voltage = clamp_to_2_decimals(vs);

        let mut state = state.write().await;
        state.batt_voltage = clamped_voltage;
        state.power_good = power_good.is_low();
        state.charge_status = charge_status.is_low();
        state.low_batt = low_batt.is_low();
    }
}

use defmt::info;
use embassy_stm32::adc::{Adc, SampleTime};
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{ADC1, PC0, PC1, PC2, PC3, PC5};
use embassy_time::Timer;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use crate::state;

#[embassy_executor::task]
pub async fn batt_voltage_monitor_task(low_batt_pin: Peri<'static, PC3>,
                                       power_good_pin: Peri<'static, PC0>,
                                       charge_status_pin: Peri<'static, PC5>,
                                       bat_mon_en_pin: Peri<'static, PC2>,
                                       adc1: Peri<'static, ADC1>,
                                       mut adc_pin: Peri<'static, PC1>,
                                       state: &'static Mutex<ThreadModeRawMutex, state::State>,
    ) {
    let low_batt = Input::new(low_batt_pin, Pull::Up);
    let power_good = Input::new(power_good_pin, Pull::Up);
    let charge_status = Input::new(charge_status_pin, Pull::Up);
    
    let mut adc_bat_v = Adc::new(adc1);
    let mut bat_mon_en = Output::new(bat_mon_en_pin, Level::Low, Speed::VeryHigh);
    
    let mut vrefint_channel = adc_bat_v.enable_vrefint();
    
    let mut vrefint_sum = 0u32;
    for _ in 0..10 {
        let vref = adc_bat_v.blocking_read(&mut vrefint_channel, SampleTime::CYCLES640_5);
        vrefint_sum += vref as u32;
        Timer::after_millis(10).await;
    }
    let vrefint_cal = (vrefint_sum / 10) as u16;
    info!("VREFINT calibration: {}", vrefint_cal);
    
    let convert_to_millivolts = |sample: f32| {
        // MIN   TYP   MAX
        //1.182 1.212 1.232
        const VREFINT_MV: f32 = 12.120; // mV
        
        (sample * VREFINT_MV / vrefint_cal as f32) as f32
    };

    loop {
        bat_mon_en.set_high();       
        Timer::after_millis(50).await;
        let measured = adc_bat_v.blocking_read(&mut adc_pin, SampleTime::CYCLES640_5);
        
        let voltage_mv = convert_to_millivolts(measured as f32);
        
        
        bat_mon_en.set_low();
        Timer::after_millis(1000).await;

        let mut state = state.lock().await;
        state.batt_voltage = voltage_mv;
        state.power_good = power_good.is_low();
        state.charge_status = charge_status.is_low();
        state.low_batt = low_batt.is_low();
        
        if low_batt.is_low() {
            info!("Low battery");
        }
    }
}

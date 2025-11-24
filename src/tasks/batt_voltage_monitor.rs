use defmt::info;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Level, Output, Pull, Speed};
use embassy_stm32::pac::ADC1;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{ADC1, PC0, PC1, PC2, PC3, PC5};
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn batt_voltage_monitor_task(low_batt_pin: Peri<'static, PC3>,
                                       power_good_pin: Peri<'static, PC0>,
                                       charge_status_pin: Peri<'static, PC5>,
                                       bat_mon_en_pin: Peri<'static, PC2>,
                                       adc1: Peri<'static, ADC1>,
                                       mut adc_pin: Peri<'static, PC1>,
    ) {
    let mut low_batt = Input::new(low_batt_pin, Pull::Up);
    let mut power_good = Input::new(power_good_pin, Pull::Up);
    let mut charge_status = Input::new(charge_status_pin, Pull::Up);
    let mut adc_bat_v = Adc::new(adc1);
    let mut bat_mon_en = Output::new(bat_mon_en_pin, Level::Low, Speed::VeryHigh);
    let mut vrefint_channel = adc_bat_v.enable_vrefint();
    loop {
        let convert_to_millivolts = |sample, vrefint| {
            // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
            // 6.3.24 Reference voltage
            const VREFINT_MV: u32 = 1210; // mV
        
            (u32::from(sample) * VREFINT_MV / u32::from(vrefint)) as u16
        };


        bat_mon_en.set_high();
        let vrefint = adc_bat_v.blocking_read(&mut vrefint_channel, SampleTime::CYCLES160_5);
        let measured = adc_bat_v.blocking_read(&mut adc_pin, SampleTime::CYCLES160_5);
        info!("Measured: {}", measured);
        info!("Converted to millivolts: {}", convert_to_millivolts(measured, vrefint));
        Timer::after_millis(1000).await;
        


        // let measured = adc_bat_v.blocking_read(&mut adc_pin);
        // info!("Measured: {}", measured);
        // info!("Converted to millivolts: {}", convert_to_millivolts(measured));

        // // print low battery status
        // if low_batt.is_low() {
        //     info!("Low battery");
        // } else {
        //     info!("Battery OK");
        // }

        // // print power good status
        // if power_good.is_low() {
        //     info!("Power good");
        // } else {
        //     info!("Power not good");
        // }

        // // print charge status
        // if charge_status.is_low() {
        //     info!("Charging");
        // } else {
        //     info!("Not charging");
        // }
    }
}

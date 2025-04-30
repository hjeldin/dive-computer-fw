use defmt::info;
use embassy_stm32::adc::Adc;
use embassy_stm32::gpio::{Input, Output};
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{ADC1, PC1};
use embassy_time::Timer;

#[embassy_executor::task]
pub async fn batt_voltage_monitor_task(low_batt: &'static mut Input<'static>,
                                       power_good: &'static mut Input<'static>,
                                       charge_status: &'static mut Input<'static>,
                                       adc_bat_v: &'static mut Adc<'static, ADC1>,
                                       bat_mon_en: &'static mut Output<'static>,
                                       adc_pin: &'static mut Peri<'static, PC1>,
    ) {
    let mut vrefint_channel = adc_bat_v.enable_vrefint();
    loop {
        Timer::after_millis(10000).await;
        bat_mon_en.set_low();
        let vrefint = adc_bat_v.blocking_read(&mut vrefint_channel);

        let convert_to_millivolts = |sample| {
            // From http://www.st.com/resource/en/datasheet/DM00071990.pdf
            // 6.3.24 Reference voltage
            const VREFINT_MV: u32 = 1210; // mV

            (u32::from(sample) * VREFINT_MV / u32::from(vrefint)) as u16
        };

        let measured = adc_bat_v.blocking_read(adc_pin);
        info!("Measured: {}", convert_to_millivolts(measured));

        // print low battery status
        if low_batt.is_low() {
            info!("Low battery");
        } else {
            info!("Battery OK");
        }

        // print power good status
        if power_good.is_low() {
            info!("Power good");
        } else {
            info!("Power not good");
        }

        // print charge status
        if charge_status.is_low() {
            info!("Charging");
        } else {
            info!("Not charging");
        }
    }
}

use crate::i2cdriver::I2CDriver;
use ism330dhcx::{ctrl1xl, ctrl2g, Ism330Dhcx};
use embassy_time::{Delay, Duration, Instant, Timer};
use mmc5983ma::MMC5983;

#[embassy_executor::task]
pub async fn ninedof_task(mut driver: I2CDriver<'static>) {
    let mut sixdof_sensor = Ism330Dhcx::new(&mut driver).await.unwrap();
    boot_sensor(&mut sixdof_sensor, &mut driver).await;
    loop {
        Timer::after_millis(1000).await;
        {
            let data = sixdof_sensor.get_accelerometer(&mut driver).await.unwrap();
            defmt::info!("{}", data);
        }
    }
}


async fn boot_sensor<I2C>(sensor: &mut Ism330Dhcx, i2c: &mut I2C)
where
    I2C: embedded_hal_async::i2c::I2c,
{
    // =======================================
    // CTRL3_C

    sensor.ctrl3c.set_boot(i2c, true).await.unwrap();
    sensor.ctrl3c.set_bdu(i2c, true).await.unwrap();
    sensor.ctrl3c.set_if_inc(i2c, true).await.unwrap();

    // =======================================
    // CTRL9_XL

    sensor.ctrl9xl.set_den_x(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_y(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_den_z(i2c, true).await.unwrap();
    sensor.ctrl9xl.set_device_conf(i2c, true).await.unwrap();

    // =======================================
    // CTRL1_XL

    sensor
        .ctrl1xl
        .set_accelerometer_data_rate(i2c, ctrl1xl::Odr_Xl::Hz52)
        .await.unwrap();

    sensor
        .ctrl1xl
        .set_chain_full_scale(i2c, ctrl1xl::Fs_Xl::G4)
        .await.unwrap();
    sensor.ctrl1xl.set_lpf2_xl_en(i2c, true).await.unwrap();

    // =======================================
    // CTRL2_G

    sensor
        .ctrl2g
        .set_gyroscope_data_rate(i2c, ctrl2g::Odr::Hz52)
        .await.unwrap();

    sensor
        .ctrl2g
        .set_chain_full_scale(i2c, ctrl2g::Fs::Dps500)
        .await.unwrap();

    // =======================================
    // CTRL7_G

    sensor.ctrl7g.set_g_hm_mode(i2c, true).await.unwrap();
}
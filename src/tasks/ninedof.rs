use crate::i2cdriver::I2CDriver;
use ism330dhcx::{ctrl1xl, ctrl2g, Ism330Dhcx};
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use crate::state;

#[embassy_executor::task]
pub async fn ninedof_task(mut driver: I2CDriver<'static>, state: &'static Mutex<ThreadModeRawMutex, state::State>) {
    let mut sixdof_sensor = Ism330Dhcx::new(&mut driver).await.unwrap();
    boot_sensor(&mut sixdof_sensor, &mut driver).await;
    const G_TO_MPS2: f64 = 9.80665; // Standard gravity in m/s²
    
    loop {
        Timer::after_millis(100).await;
        {
            let accel_data = sixdof_sensor.get_accelerometer(&mut driver).await.unwrap();
            let gyro_data = sixdof_sensor.get_gyroscope(&mut driver).await.unwrap();
            // Convert accelerometer from g to m/s² (SI units)
            let accel_g = accel_data.as_g();
            let accel_mps2 = [
                accel_g[0] * G_TO_MPS2,
                accel_g[1] * G_TO_MPS2,
                accel_g[2] * G_TO_MPS2,
            ];
            // defmt::info!("Accel (m/s²): {}", accel_mps2);
            // Gyroscope already in SI units (rad/s)
            // defmt::info!("Gyro (rad/s): {}", gyro_data.as_rad());
            let mut state = state.lock().await;
            state.accel_x = accel_mps2[0] as f32;
            state.accel_y = accel_mps2[1] as f32;
            state.accel_z = accel_mps2[2] as f32;
            state.gyro_x = gyro_data.as_rad()[0] as f32;
            state.gyro_y = gyro_data.as_rad()[1] as f32;
            state.gyro_z = gyro_data.as_rad()[2] as f32;
            drop(state);
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
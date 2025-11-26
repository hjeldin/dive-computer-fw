use crate::i2cdriver::I2CDriver;
use ahrs::Madgwick;
use ahrs::Ahrs;
use ism330dhcx::{ctrl1xl, ctrl2g, Ism330Dhcx};
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::rwlock::RwLock;
use libm::atan2;
use crate::mmc5983ma::{ContinuousMeasurementFreq, MMC5983, PeriodicSetInterval};
use nalgebra::Vector3;
use crate::state;

#[embassy_executor::task]
pub async fn ninedof_task(
    mut sixdof_driver: I2CDriver<'static>, 
    mut threedof_driver: I2CDriver<'static>, 
    state: &'static RwLock<ThreadModeRawMutex, state::State>
) {
    let mut sixdof_sensor = Ism330Dhcx::new(&mut sixdof_driver).await.unwrap();
    let mut threedof_sensor = MMC5983::new(threedof_driver, Delay, 0x30);

    // let mut ahrs = Madgwick::default();
    let mut ahrs = Madgwick::new(1.0, 0.04);
    // let mut threedof_sensor = MMC5983::new(threedof_driver, Delay);

    boot_6dof_sensor(&mut sixdof_sensor, &mut sixdof_driver).await;
    boot_3dof_sensor(&mut threedof_sensor).await;

    const G_TO_MPS2: f64 = 9.80665; // Standard gravity in m/s²
    
    loop {
        Timer::after_millis(1000).await;
        {
            let accel_data = sixdof_sensor.get_accelerometer(&mut sixdof_driver).await.unwrap();
            let gyro_data = sixdof_sensor.get_gyroscope(&mut sixdof_driver).await.unwrap();
            // Convert accelerometer from g to m/s² (SI units)
            let accel_g = accel_data.as_g();
            let accel_mps2 = [
                accel_g[0] * G_TO_MPS2,
                accel_g[1] * G_TO_MPS2,
                accel_g[2] * G_TO_MPS2,
            ];
            // threedof_sensor.reset().await.unwrap();
            // Timer::after_millis(10).await;
            let mag_data = threedof_sensor.do_measurement_raw().await.unwrap();


            let mut loc_state = state.write().await;
            loc_state.accel_x = accel_mps2[0] as f32;
            loc_state.accel_y = accel_mps2[1] as f32;
            loc_state.accel_z = accel_mps2[2] as f32;
            loc_state.gyro_x = gyro_data.as_rad()[0] as f32;
            loc_state.gyro_y = gyro_data.as_rad()[1] as f32;
            loc_state.gyro_z = gyro_data.as_rad()[2] as f32;
            loc_state.mag_x = mag_data.x as f32;
            loc_state.mag_y = mag_data.y as f32;
            loc_state.mag_z = mag_data.z as f32;
            drop(loc_state);

            let temp = threedof_sensor.get_temp_c().await.unwrap();

            let mut accelerometer = Vector3::new(accel_mps2[0], accel_mps2[1], accel_mps2[2]);
            let gyroscope = Vector3::new(gyro_data.as_rad()[0], gyro_data.as_rad()[1], gyro_data.as_rad()[2]);
            let gyro_deg = Vector3::new(gyro_data.as_dps()[0], gyro_data.as_dps()[1], gyro_data.as_dps()[2]);
            let magnetometer_raw = Vector3::new(mag_data.x as f64, mag_data.y as f64, mag_data.z as f64);
            let magnetometer_offset = Vector3::new(131072.0, 131072.0, 131072.0);

            let mut magnetometer_normalized = (magnetometer_raw - magnetometer_offset) * 0.00625;
            magnetometer_normalized.y = -magnetometer_normalized.y;
            magnetometer_normalized.z = -magnetometer_normalized.z;
            magnetometer_normalized = magnetometer_normalized.normalize();

            // defmt::info!("Temp: {}", temp.unwrap_or(0.0));
            // defmt::info!("Accel (x, y, z): {}, {}, {}", accelerometer.x, accelerometer.y, accelerometer.z);
            // defmt::info!("Gyro (x, y, z): {}, {}, {}", gyroscope.x, gyroscope.y, gyroscope.z);
            // defmt::info!("Mag (x, y, z, norm): {}, {}, {}, {}", magnetometer_normalized.x, magnetometer_normalized.y, magnetometer_normalized.z, magnetometer_normalized.magnitude());
            // defmt::info!("Mag heading: {}", calculate_heading(magnetometer_normalized.x, magnetometer_normalized.y) as f32);

            accelerometer = accelerometer.normalize();
            // let quaternion = ahrs.update(&gyroscope, &accelerometer, &magnetometer_normalized).unwrap();
            // // let quaternion = ahrs.update_imu(&gyroscope, &accelerometer).unwrap();
            // let euler = quaternion.euler_angles();
            // defmt::info!("Roll: {}, Pitch: {}, Yaw: {}", euler.0, euler.1, euler.2);
        }
    }
}


async fn boot_6dof_sensor<I2C>(sensor: &mut Ism330Dhcx, i2c: &mut I2C)
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

async fn boot_3dof_sensor(sensor: &mut MMC5983<I2CDriver<'static>, Delay>)
{
    sensor.init().await.unwrap();
    sensor.reset().await.unwrap();
    // sensor.set_cmm_mode(ContinuousMeasurementFreq::Hz50, PeriodicSetInterval::Off).await.unwrap();
}

fn calculate_heading(mag_x: f64, mag_y: f64) -> f64 {
    let heading = atan2(mag_y, mag_x);
    return heading.to_degrees();
}
use crate::i2cdriver::I2CDriver;
use ism330dhcx::{ctrl1xl, ctrl2g, Ism330Dhcx};
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use libm::atan2;
use mmc5983ma::MMC5983;
use crate::state;

#[embassy_executor::task]
pub async fn ninedof_task(mut sixdof_driver: I2CDriver<'static>, mut threedof_driver: I2CDriver<'static>, state: &'static Mutex<ThreadModeRawMutex, state::State>) {
    let mut sixdof_sensor = Ism330Dhcx::new(&mut sixdof_driver).await.unwrap();
    let mut threedof_sensor = MMC5983::new(threedof_driver, Delay, 0x30);
    // let mut threedof_sensor = MMC5983::new(threedof_driver, Delay);

    boot_6dof_sensor(&mut sixdof_sensor, &mut sixdof_driver).await;
    boot_3dof_sensor(&mut threedof_sensor).await;

    const G_TO_MPS2: f64 = 9.80665; // Standard gravity in m/s²
    
    loop {
        Timer::after_millis(100).await;
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
            let mut state = state.lock().await;
            state.accel_x = accel_mps2[0] as f32;
            state.accel_y = accel_mps2[1] as f32;
            state.accel_z = accel_mps2[2] as f32;
            state.gyro_x = gyro_data.as_rad()[0] as f32;
            state.gyro_y = gyro_data.as_rad()[1] as f32;
            state.gyro_z = gyro_data.as_rad()[2] as f32;
            
            let mag_data = threedof_sensor.do_measurement_raw().await.unwrap();
            let mag_heading = calculate_heading(mag_data.x, mag_data.y, mag_data.z);
            state.mag_x = mag_data.x as f32;
            state.mag_y = mag_data.y as f32;
            state.mag_z = mag_data.z as f32;
            state.mag_heading = mag_heading.unwrap_or(0.0) as f32;
            drop(state);

            let temp = threedof_sensor.get_temp_c().await.unwrap();
            defmt::info!("Temp: {}", temp.unwrap_or(0.0));
            defmt::info!("Accel (m/s²): {}", accel_mps2);
            defmt::info!("Gyro (rad/s): {}", gyro_data.as_rad());
            defmt::info!("Mag (x, y, z): {}, {}, {}", mag_data.x, mag_data.y, mag_data.z);
            defmt::info!("Mag heading: {}", mag_heading.unwrap_or(0.0));
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
}

fn calculate_heading(
    current_x: u32,
    current_y: u32,
    _current_z: u32,
) -> Option<f64> {
    // According to MMC5983MA datasheet:
    // - 18-bit operation: values range from 0 to 262143 (2^18 - 1)
    // - Null field output: 131072 counts (center value)
    // - Sensitivity: 16384 counts/Gauss for 18-bit operation
    // - Full scale range: ±8 Gauss
    
    const MAX_VALUE: u32 = 262_143; // 2^18 - 1
    const MID_VALUE: f64 = 131_072.0; // Center value for 18-bit operation
    const DEG_PER_RAD: f64 = 57.29577951308232; // 180 / π, more precise conversion

    // Validate input values are within valid range
    if current_x == 0 || current_x >= MAX_VALUE {
        return None;
    }
    
    if current_y == 0 || current_y >= MAX_VALUE {
        return None;
    }

    // Convert 18-bit unsigned counts to signed values centered at zero
    // This gives us the magnetic field strength relative to the null field output
    let mag_x = current_x as f64 - MID_VALUE;
    let mag_y = current_y as f64 - MID_VALUE;

    // Calculate heading using atan2 for proper quadrant handling
    // Standard compass convention: atan2(x, y) where:
    // - 0° = North (positive Y)
    // - 90° = East (positive X)
    // - 180° = South (negative Y)
    // - 270° = West (negative X)
    // Note: atan2 returns values from -π to π, we convert to 0-360° range
    let heading_rad = atan2(mag_x, mag_y);
    let mut heading_deg = heading_rad * DEG_PER_RAD;
    
    // Normalize to 0-360° range (atan2 returns -180° to +180°)
    if heading_deg < 0.0 {
        heading_deg += 360.0;
    }

    Some(heading_deg)
}
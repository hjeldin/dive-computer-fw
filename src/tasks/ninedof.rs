use crate::i2cdriver::I2CDriver;
use ahrs::Madgwick;
use ahrs::Ahrs;
use ism330dhcx::{ctrl1xl, ctrl2g, Ism330Dhcx};
use embassy_time::{Delay, Duration, Instant, Timer};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::rwlock::RwLock;
// micromath's atan2 is f32 throughout; libm's trig works in f64 internally,
// which is software floating point on the M4F
use libm::sqrtf;
use micromath::F32Ext;
use crate::mmc5983ma::{ContinuousMeasurementFreq, MMC5983, PeriodicSetInterval};
use nalgebra::Vector3;
use crate::state;
use crate::{LCD_WAKE, LOW_POWER_MODE};
use core::sync::atomic::Ordering;
use defmt::info;

/// Hard-iron calibration of the magnetometer, in raw counts per axis (x, y, z).
/// Centre of each axis' range; replace with the `offset` that the
/// calibration log prints. 131072 is the midpoint of the 18-bit output.
const MAG_OFFSET: [f32; 3] = [131072.0, 131072.0, 131072.0];
/// Half of each axis' range; replace with the `half range` from the log.
/// Equal values leave the axes unscaled.
const MAG_HALF_RANGE: [f32; 3] = [1.0, 1.0, 1.0];

/// Set to true, flash, and rotate the device slowly through every
/// orientation for about 30 s. The task then samples at 10 Hz and logs the
/// min and max of each magnetometer axis with the values for the constants
/// above. Set back to false afterwards.
const MAG_CALIBRATION: bool = true;

#[embassy_executor::task]
pub async fn ninedof_task(
    mut sixdof_driver: I2CDriver<'static>, 
    mut threedof_driver: I2CDriver<'static>, 
    state: &'static RwLock<ThreadModeRawMutex, state::State>
) {
    let mut sixdof_sensor = Ism330Dhcx::new(&mut sixdof_driver).await.unwrap();
    let mut threedof_sensor = MMC5983::new(threedof_driver, Delay, 0x30);

    // let mut ahrs = Madgwick::default();
    let mut ahrs = Madgwick::<f32>::new(1.0, 0.04);
    // let mut threedof_sensor = MMC5983::new(threedof_driver, Delay);

    boot_6dof_sensor(&mut sixdof_sensor, &mut sixdof_driver).await;
    boot_3dof_sensor(&mut threedof_sensor).await;

    const G_TO_MPS2: f32 = 9.80665; // Standard gravity in m/s²
    let loop_interval_ms = if MAG_CALIBRATION { 100 } else { 1000 };
    let mut mag_min = [f32::MAX; 3];
    let mut mag_max = [f32::MIN; 3];
    let mut samples: u32 = 0;

    loop {
        // Check for low power mode
        if LOW_POWER_MODE.load(Ordering::Relaxed) {
            info!("[Ninedof] Low power mode - disabling sensors");
            // Wait for low power mode to be cleared
            loop {
                Timer::after_millis(100).await;
                if !LOW_POWER_MODE.load(Ordering::Relaxed) {
                    info!("[Ninedof] Waking from low power mode");
                    break;
                }
            }
            continue;
        }
        
        Timer::after_millis(loop_interval_ms).await;
        {
            let accel_data = sixdof_sensor.get_accelerometer(&mut sixdof_driver).await.unwrap();
            let gyro_data = sixdof_sensor.get_gyroscope(&mut sixdof_driver).await.unwrap();
            // The M4F FPU is single-precision only, so all maths stays in f32
            let accel_g = accel_data.as_g_f32();
            let gyro_rad = gyro_data.as_rad_f32();
            // Convert accelerometer from g to m/s² (SI units)
            let accel_mps2 = [
                accel_g[0] * G_TO_MPS2,
                accel_g[1] * G_TO_MPS2,
                accel_g[2] * G_TO_MPS2,
            ];
            // threedof_sensor.reset().await.unwrap();
            // Timer::after_millis(10).await;
            let mag_data = threedof_sensor.do_measurement_raw().await.unwrap();


            let mut loc_state = state.write().await;
            loc_state.accel_x = accel_mps2[0];
            loc_state.accel_y = accel_mps2[1];
            loc_state.accel_z = accel_mps2[2];
            loc_state.gyro_x = gyro_rad[0];
            loc_state.gyro_y = gyro_rad[1];
            loc_state.gyro_z = gyro_rad[2];
            loc_state.mag_x = mag_data.x as f32;
            loc_state.mag_y = mag_data.y as f32;
            loc_state.mag_z = mag_data.z as f32;
            drop(loc_state);

            let temp = threedof_sensor.get_temp_c().await.unwrap();

            let mut accelerometer = Vector3::new(accel_mps2[0], accel_mps2[1], accel_mps2[2]);
            let gyroscope = Vector3::new(gyro_rad[0], gyro_rad[1], gyro_rad[2]);
            let magnetometer_raw = Vector3::new(mag_data.x as f32, mag_data.y as f32, mag_data.z as f32);
            if MAG_CALIBRATION {
                log_mag_calibration(&magnetometer_raw, &mut mag_min, &mut mag_max, &mut samples);
            }

            // Centre and scale each axis, so Earth's field traces a sphere
            // around the origin; the length is normalised away below
            let mut magnetometer_normalized = Vector3::new(
                (magnetometer_raw.x - MAG_OFFSET[0]) / MAG_HALF_RANGE[0],
                (magnetometer_raw.y - MAG_OFFSET[1]) / MAG_HALF_RANGE[1],
                (magnetometer_raw.z - MAG_OFFSET[2]) / MAG_HALF_RANGE[2],
            );
            magnetometer_normalized.y = -magnetometer_normalized.y;
            magnetometer_normalized.z = -magnetometer_normalized.z;
            magnetometer_normalized = magnetometer_normalized.normalize();

            // defmt::info!("Temp: {}", temp.unwrap_or(0.0));
            // defmt::info!("Accel (x, y, z): {}, {}, {}", accelerometer.x, accelerometer.y, accelerometer.z);
            // defmt::info!("Gyro (x, y, z): {}, {}, {}", gyroscope.x, gyroscope.y, gyroscope.z);
            // defmt::info!("Mag (x, y, z, norm): {}, {}, {}, {}", magnetometer_normalized.x, magnetometer_normalized.y, magnetometer_normalized.z, magnetometer_normalized.magnitude());
            // defmt::info!("Mag heading: {}", calculate_heading(magnetometer_normalized.x, magnetometer_normalized.y) as f32);

            accelerometer = accelerometer.normalize();

            let heading = tilt_compensated_heading(&accelerometer, &magnetometer_normalized);
            state.write().await.mag_heading = heading;
            LCD_WAKE.signal(());
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

/// Tracks the range of each raw magnetometer axis and logs it once a second
/// (every 10 samples at the calibration rate).
fn log_mag_calibration(raw: &Vector3<f32>, min: &mut [f32; 3], max: &mut [f32; 3], samples: &mut u32) {
    for axis in 0..3 {
        min[axis] = min[axis].min(raw[axis]);
        max[axis] = max[axis].max(raw[axis]);
    }
    *samples += 1;
    if *samples % 10 != 0 {
        return;
    }
    let offset = |axis: usize| (max[axis] + min[axis]) / 2.0;
    let half_range = |axis: usize| (max[axis] - min[axis]) / 2.0;
    info!(
        "[MagCal] {} samples, min ({}, {}, {}), max ({}, {}, {})",
        *samples, min[0], min[1], min[2], max[0], max[1], max[2]
    );
    info!(
        "[MagCal] MAG_OFFSET: [{}, {}, {}], MAG_HALF_RANGE: [{}, {}, {}]",
        offset(0), offset(1), offset(2), half_range(0), half_range(1), half_range(2)
    );
}

/// Compass heading in degrees, 0..360, corrected for the device's tilt.
///
/// Roll and pitch come from gravity in `accel`; the magnetic field in `mag`
/// is rotated onto the horizontal plane before taking the angle
/// (NXP AN4248). Both vectors must be in the same axes. Turning clockwise,
/// seen from above, increases the heading.
///
/// The device's forward edge points along the sensors' -x axis, so the
/// angle of +x is turned by 180 degrees.
fn tilt_compensated_heading(accel: &Vector3<f32>, mag: &Vector3<f32>) -> f32 {
    // Sine and cosine of roll and pitch, read straight off the gravity
    // vector instead of going through the angles: no sin/cos calls, and
    // micromath's approximate sin/cos would add up to 3 degrees of error
    let yz = sqrtf(accel.y * accel.y + accel.z * accel.z);
    let (sin_roll, cos_roll) = (accel.y / yz, accel.z / yz);
    let xyz = sqrtf(accel.x * accel.x + yz * yz);
    let (sin_pitch, cos_pitch) = (-accel.x / xyz, yz / xyz);

    let horizontal_x = mag.x * cos_pitch
        + mag.y * sin_pitch * sin_roll
        + mag.z * sin_pitch * cos_roll;
    let horizontal_y = mag.z * sin_roll - mag.y * cos_roll;

    // atan2 gives -180..180, so this lands in 0..360
    F32Ext::atan2(horizontal_y, horizontal_x).to_degrees() + 180.0
}

fn calculate_heading(mag_x: f32, mag_y: f32) -> f32 {
    let heading = F32Ext::atan2(mag_y, mag_x);
    return heading.to_degrees();
}
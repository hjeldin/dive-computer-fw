use crate::i2cdriver::I2CDriver;
use ism330dhcx::{ctrl1xl, ctrl2g, Ism330Dhcx};
use embassy_time::{Delay, Duration, Instant, Timer};
use mmc5983ma::MMC5983;
use libm::atan;

#[embassy_executor::task]
pub async fn threedof_task(mut driver: I2CDriver<'static>) {
    let mut threedof_sensor = MMC5983::new(&mut driver, Delay, 0x30);
    Timer::after_millis(1000).await;
    threedof_sensor.init().await.unwrap();
    threedof_sensor.reset().await.unwrap();

    loop {
        Timer::after_millis(1000).await;
        unsafe {
            let data = threedof_sensor.read_raw_measurement_18().await.unwrap();
            defmt::info!("HEADING: {}", calculate_heading(data.x, data.y, data.z));
        }
        defmt::info!("TEMP: {}", threedof_sensor.get_temp_c().await.unwrap());
    }
}

fn calculate_heading(
    current_x: u32,
    current_y: u32,
    current_z: u32,
) -> Option<f64> {
    const MAX_VALUE: u32 = 262_143;
    const MID_VALUE: f64 = 131_072.0;
    const DEG_PER_RAD: f64 = 57.2958; // Conversion factor: radians to degrees

    let mut good = true;

    // Check if the values are within the valid range
    // if current_x == 0 || current_x >= MAX_VALUE {
    //     good = false;
    // }
    //
    // if current_y == 0 || current_y >= MAX_VALUE {
    //     good = false;
    // }
    //
    // if current_z == 0 || current_z >= MAX_VALUE {
    //     good = false;
    // }

    if good {
        // Normalize the values
        let mut normalized_x = current_x as f64 - MID_VALUE;
        normalized_x /= MID_VALUE;

        let mut normalized_y = current_y as f64 - MID_VALUE;
        normalized_y /= MID_VALUE;

        let mut normalized_z = current_z as f64 - MID_VALUE;
        normalized_z /= MID_VALUE;

        defmt::info!(
            "X axis raw value: {}\tY axis raw value: {}\tZ axis raw value: {}",
            current_x, current_y, current_z
        );

        // Calculate heading
        let heading = if normalized_y != 0.0 {
            if normalized_x < 0.0 {
                if normalized_y > 0.0 {
                    // Quadrant 1
                    DEG_PER_RAD * atan(-normalized_x / normalized_y)
                } else {
                    // Quadrant 2
                    DEG_PER_RAD * atan(-normalized_x / normalized_y) + 180.0
                }
            } else {
                if normalized_y < 0.0 {
                    // Quadrant 3
                    DEG_PER_RAD * atan(-normalized_x / normalized_y) + 180.0
                } else {
                    // Quadrant 4
                    360.0 - DEG_PER_RAD * atan(normalized_x / normalized_y)
                }
            }
        } else {
            // Handle infinite slopes
            if normalized_x > 0.0 {
                270.0
            } else {
                90.0
            }
        };

        Some(heading)
    } else {
        None // Invalid input values
    }
}
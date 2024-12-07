use core::convert::TryInto;

use embassy_stm32::{i2c::I2c, mode::Async};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::Timer;

use crate::i2cdriver::I2CDriver;

#[allow(dead_code)]
mod bmp280regs {    
    pub const ADDR: u8 = 0x53;

    macro_rules! mcpregs {
        ($($name:ident : $val:expr),* $(,)?) => {
            $(
                pub const $name: u8 = $val;
            )*

            pub fn regname(reg: u8) -> &'static str {
                match reg {
                    $(
                        $val => stringify!($name),
                    )*
                    _ => panic!("bad reg"),
                }
            }
        }
    }

    mcpregs! {
        ID: 0xD0,
        RESET: 0xE0,
        CTRLHUM: 0xF2,
        STATUS: 0xF3,
        CTRLMEAS: 0xF4,
        CONFIG: 0xF5,
        PRESSMSB: 0xF7,
        PRESSLSB: 0xF8,
        PRESSXLSB: 0xF9,
        TEMPMSB: 0xFA,
        TEMPLSB: 0xFB,
        TEMPXLSB: 0xFC,
        HUMMSB: 0xFD,
        HUMLSB: 0xFE,
        CALIB_T11: 0x88,
        CALIB_T12: 0x89,
        CALIB_T21: 0x8A,
        CALIB_T22: 0x8B,
        CALIB_T31: 0x8C,
        CALIB_T32: 0x8D,
    }
}


fn bme280_compensate_t_int32(adc_t: i32, dig_t1: u16, dig_t2: i16, dig_t3: i16) -> (i32, i32) {
    let var1: i32 = (((adc_t >> 3) - ((dig_t1 as i32) << 1)) * dig_t2 as i32) >> 11;
    let var2: i32 = (((((adc_t >> 4) - (dig_t1 as i32))
        * ((adc_t >> 4) - (dig_t1 as i32)))
        >> 12)
        * dig_t3 as i32)
        >> 14;

    let t = var1 + var2;
    ((t * 5 + 128) >> 8, t) // Temperature in 0.01 °C
}

// Function to compensate pressure in Pa
fn bme280_compensate_p_int32(adc_p: i32, t_fine: i32) -> u32 {
    // Replace these values with the calibration parameters from the BME280
    let dig_p1: u32 = 36477; // Example calibration value
    let dig_p2: i32 = -10685;
    let dig_p3: i32 = 3024;
    let dig_p4: i32 = 2855;
    let dig_p5: i32 = 140;
    let dig_p6: i32 = -7;
    let dig_p7: i32 = 15500;
    let dig_p8: i32 = -14600;
    let dig_p9: i32 = 6000;

    let mut var1: i32;
    let mut var2: i32;
    let mut p: u32;

    var1 = (((t_fine >> 1) as i32) - 64000) as i32;
    var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) * dig_p6) >> 2;
    var2 += ((var1 * dig_p5) << 1);
    var2 = (var2 >> 2) + (dig_p4 << 16);
    var1 = (((dig_p3 * ((var1 >> 2) * (var1 >> 2)) >> 13) >> 3) + ((dig_p2 * var1) >> 1)) >> 18;
    var1 = (((32768 + var1) * dig_p1 as i32) >> 15) as i32;

    if var1 == 0 {
        return 0; // Avoid division by zero
    }

    p = (((1048576 - adc_p) as i32 - (var2 >> 12)) * 3125) as u32;
    if p >= 0x80000000 {
        p = ((p >> 1) / var1 as u32) << 1;
    } else {
        p = (p * 2) / var1 as u32;
    }

    var1 = ((dig_p9 * ((p as i32 >> 3) * (p as i32 >> 3)) >> 13) >> 12) as i32;
    var2 = (((p as i32 >> 2) * dig_p8) >> 13) as i32;
    p = p + ((var1 + var2 + dig_p7) >> 4) as u32;

    p
}

fn bme280_compensate_h_double32(adc_h: i32, t_fine: i32) -> f64 {
    let mut h: f64 = 0.0;
    let dig_h1 = 76800.0;
    h = t_fine as f64 - dig_h1;

    h
}

pub enum Bmp280CalibData {
    TEMPERATURE,
    PRESSURE,
    HUMIDITY
}

pub struct BMP280<'a> {
    driver: I2CDriver<'a>,
    t_fine: i32,
    dig_t1: u16,
    dig_t2: i16,
    dig_t3: i16,
}

impl<'a> BMP280<'a> {
    pub fn new(device: I2CDriver<'a>) -> Self {
        BMP280 { driver: device, t_fine: 0, dig_t1: 0, dig_t2: 0, dig_t3: 0 }
    }

    pub async fn init(&mut self) {
        let assigned_id = 0x60;
        loop {
            let mut id: u16 = 0;
            let mut buffer: [u8; 1] = [0; 1];
            self.driver.write_read(&[bmp280regs::ID], &mut buffer).await;

            id = buffer[0] as u16;

    
            if assigned_id == id {
                defmt::info!("[BMP280] ID assigned = {}", id);
                return;
            } else {
                defmt::info!("[BMP280] Invalid id assigned = {}", id);
                Timer::after_millis(1000).await;
            }
        }
    }

    pub async fn get_status(&mut self) {
        let mut buffer: [u8; 8] = [0; 8];
        let result = self.driver.write_read(&[bmp280regs::STATUS], &mut buffer).await;
        defmt::info!("[BMP280] Status = {}", buffer);
    }

    pub async fn get_press(&mut self) {
        let mut buffer_msb: [u8; 1] = [0; 1];
        let mut buffer_lsb: [u8; 1] = [0; 1];
        let mut buffer_xlsb: [u8; 1] = [0; 1];

        let result_msb = self.driver.write_read(&[bmp280regs::PRESSMSB], &mut buffer_msb).await;
        let result_lsb = self.driver.write_read(&[bmp280regs::PRESSLSB], &mut buffer_lsb).await;
        let result_xlsb = self.driver.write_read(&[bmp280regs::PRESSXLSB], &mut buffer_xlsb).await;

        let value: i32 = (buffer_msb[0] as i32) << 16 | (buffer_lsb[0] as i32) << 8 | (buffer_xlsb[0] as i32);

        let compensated_value = bme280_compensate_p_int32(value, self.t_fine);


        defmt::info!("[BMP280] Pressure MSB = {} - LSB = {} - XLSB = {} - VALUE = {} - COMPENSATED VALUE = {}", buffer_msb, buffer_lsb, buffer_xlsb, value, compensated_value);
        
    }

    pub async fn get_temp(&mut self) {
        let mut buffer_msb: [u8; 1] = [0; 1];
        let mut buffer_lsb: [u8; 1] = [0; 1];
        let mut buffer_xlsb: [u8; 1] = [0; 1];

        let result_msb = self.driver.write_read(&[bmp280regs::TEMPMSB], &mut buffer_msb).await;
        let result_lsb = self.driver.write_read(&[bmp280regs::TEMPLSB], &mut buffer_lsb).await;
        let result_xlsb = self.driver.write_read(&[bmp280regs::TEMPXLSB], &mut buffer_xlsb).await;

        let value: i32 = (buffer_msb[0] as i32) << 16 | (buffer_lsb[0] as i32) << 8 | (buffer_xlsb[0] as i32);
        let compensated_value = bme280_compensate_t_int32(value, self.dig_t1, self.dig_t2, self.dig_t3);
        self.t_fine = compensated_value.1;
        

        defmt::info!("[BMP280]T-fine = {} - Temperature MSB = {} - LSB = {} - XLSB = {} - VALUE = {} - COMPENSATED VALUE = {} ", self.t_fine, buffer_msb, buffer_lsb, buffer_xlsb, value, compensated_value);
        
    }

    pub async fn get_hum(&mut self) {
        let mut buffer_msb: [u8; 1] = [0; 1];
        let mut buffer_lsb: [u8; 1] = [0; 1];

        let result_msb = self.driver.write_read(&[bmp280regs::HUMMSB], &mut buffer_msb).await;
        let result_lsb = self.driver.write_read(&[bmp280regs::HUMLSB], &mut buffer_lsb).await;

        let value: i32 = (buffer_msb[0] as i32) << 8 | (buffer_lsb[0] as i32) << 0;
        let compensated_value = bme280_compensate_h_double32(value, self.t_fine);

        defmt::info!("[BMP280] Humidity MSB = {} - LSB = {} -- VALUE = {} - COMPENSATED VALUE = {} ", buffer_msb, buffer_lsb, value, compensated_value);
    }

    pub async fn set_mode(&mut self, mode: u8) {
        let mut buffer_hum: [u8; 1] = [0; 1];
        let mut buffer_meas: [u8; 1] = [0; 1];
        let mut buffer_config: [u8; 1] = [0; 1];
        buffer_hum[0] = 0b00000001;
        buffer_meas[0] = 0b00100111;
        buffer_config[0] = 0b01000000;
        let result_hum = self.driver.write_bytes(&[bmp280regs::CTRLHUM, buffer_hum[0]]).await;
        let result_meas = self.driver.write_bytes(&[bmp280regs::CTRLMEAS, buffer_meas[0]]).await;
        let result_config = self.driver.write_bytes(&[bmp280regs::CONFIG, buffer_config[0]]).await;
        Timer::after_millis(1000).await;
    }

    pub async fn get_calibration_data(&mut self, calib_data: Bmp280CalibData) {
        match calib_data {
            Bmp280CalibData::HUMIDITY => {

            }
            Bmp280CalibData::PRESSURE => {

            }
            Bmp280CalibData::TEMPERATURE => {
                let mut dig_t11_buff: [u8; 2] = [0; 2];

                let mut dig_t21_buff: [u8; 2] = [0; 2];

                let mut dig_t31_buff: [u8; 2] = [0; 2];

                self.driver.write_read(&[bmp280regs::CALIB_T11], &mut dig_t11_buff).await;                

                self.driver.write_read(&[bmp280regs::CALIB_T21], &mut dig_t21_buff).await;

                self.driver.write_read(&[bmp280regs::CALIB_T31], &mut dig_t31_buff).await;

                let tmp_t1 = (dig_t11_buff[0] as u16) << 8 | (dig_t11_buff[1] as u16);
                self.dig_t1 = (tmp_t1 >> 8) | (tmp_t1 << 8);

                let tmp_t2 = (dig_t21_buff[0] as u16) << 8 | (dig_t21_buff[1] as u16);
                self.dig_t2 = ((tmp_t2 >> 8) | (tmp_t2 << 8)) as i16;

                let tmp_t3 = (dig_t31_buff[0] as u16) << 8 | (dig_t31_buff[1] as u16);
                self.dig_t3 = ((tmp_t3 >> 8) | (tmp_t3 << 8)) as i16;

                defmt::info!("[BMP280] Buff t1: {} ", dig_t11_buff);
                defmt::info!("[BMP280] Buff t2: {} ", dig_t21_buff);
                defmt::info!("[BMP280] Buff t3: {} ", dig_t31_buff);
                defmt::info!("[BMP280] Got calibration data: {} {} {}", self.dig_t1, self.dig_t2, self.dig_t3);

                Timer::after_millis(1000).await;
            }
        }

    }
}

#[embassy_executor::task]
pub async fn bmp_task(sensor: I2CDriver<'static>) {
    // Configure the LED pin as a push pull output and obtain handler.
    // On the Nucleo F091RC there's an on-board LED connected to pin PA5.
    let mut bmp280_sensor = BMP280::new(sensor);
    bmp280_sensor.init().await;
    bmp280_sensor.get_calibration_data(Bmp280CalibData::TEMPERATURE).await;
    bmp280_sensor.set_mode(0xff).await;
    loop{
        // bmp280_sensor.get_status().await;
        // bmp280_sensor.get_press().await;
        bmp280_sensor.get_temp().await;
        // bmp280_sensor.get_hum().await;
        Timer::after_millis(1000).await;
    }
    // set_opmode(i2c, 0x53, 0x00);
}
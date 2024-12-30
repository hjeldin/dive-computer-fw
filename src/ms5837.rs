use core::sync::atomic::Ordering;
use defmt::info;
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::Timer;
use static_cell::StaticCell;

use crate::i2cdriver::I2CDriver;
use crate::LOW_POWER_MODE;

#[allow(dead_code)]
mod ms5837regs {
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
        RESET: 0x1E,
        ADC_READ: 0x00,
        PROM_READ_BASE: 0xA0u8,
        READ_D1_BASE: 0x40,
        READ_D2_BASE: 0x50,
    }

    #[derive(Copy, Clone)]
    pub enum Resolution {
        Osr256 = 0,
        Osr512 = 1,
        Osr1024 = 2,
        Osr2048 = 3,
        Osr4096 = 4,
        Osr8192 = 5,
    }
}

pub struct MS5837<'a> {
    driver: I2CDriver<'a>,
    prom: [u16; 7],
    calibration_data: FactoryCalibrationData
}

#[derive(PartialEq, Debug)]
pub struct FactoryCalibrationData {
    /// Pressure sensitivity
    pressure_sensitivity: u16,
    pressure_offset: u16,
    temperature_coefficient_of_pressure_sensitivty: u16,
    temperature_coefficient_of_pressure_offset: u16,
    reference_temperature: u16,
    temperature_coefficient_of_temperature: u16,
}
use defmt::Format;
use libm::powf;
use crate::ms5837::ms5837regs::Resolution::{Osr256, Osr512};

impl Format for FactoryCalibrationData {
    fn format(&self, fmt: defmt::Formatter) {
        defmt::write!(
            fmt,
            "FactoryCalibrationData {{ pressure_sensitivity: {}, pressure_offset: {}, temperature_coefficient_of_pressure_sensitivty: {}, temperature_coefficient_of_pressure_offset: {}, reference_temperature: {}, temperature_coefficient_of_temperature: {} }}",
            self.pressure_sensitivity,
            self.pressure_offset,
            self.temperature_coefficient_of_pressure_sensitivty,
            self.temperature_coefficient_of_pressure_offset,
            self.reference_temperature,
            self.temperature_coefficient_of_temperature
        );
    }
}
impl Default for FactoryCalibrationData {
    fn default() -> Self {
        FactoryCalibrationData {
            pressure_sensitivity: 0,
            pressure_offset: 0,
            temperature_coefficient_of_pressure_sensitivty: 0,
            temperature_coefficient_of_pressure_offset: 0,
            reference_temperature: 0,
            temperature_coefficient_of_temperature: 0,
        }
    }
}

impl<'a> MS5837<'a> {
    pub fn new(device: I2CDriver<'a>) -> Self {
        MS5837 {
            driver: device,
            prom: [0; 7],
            calibration_data: Default::default()
        }
    }

    pub async fn init(&mut self) {
        let _ = self.driver.write_bytes(&[ms5837regs::RESET]).await;
        Timer::after_millis(10).await;
        info!("MS5837 reset done!");
        self.calibration_data = self.read_calibration_data().await;
        info!("MS5837 read prom! {:#?}", self.calibration_data);
    }

    // pub async fn read_prom(&mut self) {
    //     // Read PROM coefficients
    //     for i in 0..7 {
    //         let mut data = [0; 2];
    //         let _ = self
    //             .driver
    //             .write_read_bytes(&[ms5837regs::PROM_READ_BASE | (i as u8 * 2)], &mut data)
    //             .await;
    //         self.prom[i] = u16::from_be_bytes(data);
    //     }
    //     defmt::info!("PROM: {:?}", self.prom);
    // }

    pub async fn read_prom(
        &mut self
    ) {
        let mut prom_address: u8 = 0;
        for entry in self.prom.iter_mut() {
            let mut buffer = [0, 0];
            let _ = self
                .driver
                .write_read_bytes(&[ms5837regs::PROM_READ_BASE | prom_address << 1], &mut buffer)
                .await;
            *entry = u16::from_be_bytes(buffer);
            prom_address += 1;
        }

        defmt::info!("PROM: {:?}", self.prom);
    }

    fn crc4(buffer: &[u16]) -> u8 {
        let mut n_remainder: u16 = 0;
        for byte in buffer
            .iter()
            .chain([0u16].iter())
            .flat_map(|word| word.to_be_bytes())
        {
            n_remainder ^= byte as u16;
            for _ in 0..8 {
                if (n_remainder & 0x8000) != 0 {
                    n_remainder = (n_remainder << 1) ^ 0x3000;
                } else {
                    n_remainder = n_remainder << 1;
                }
            }
        }
        n_remainder = (n_remainder >> 12) & 0x000f;
        (n_remainder ^ 0x00) as u8
    }

    async fn read_calibration_data(
        &mut self,
    ) -> FactoryCalibrationData {
        self.read_prom().await;
        let expected_crc4 = ((0xF000 & self.prom[0]) >> 12) as u8;
        self.prom[0] = self.prom[0] & 0x0FFF;
        let got_crc4 = Self::crc4(&self.prom[..]);
        if expected_crc4 != got_crc4 {
            panic!("CRC4 mismatch: expected {:x}, got {:x}", expected_crc4, got_crc4);
        }
        let prom = &self.prom[1..];
        FactoryCalibrationData {
            pressure_sensitivity: prom[0],
            pressure_offset: prom[1],
            temperature_coefficient_of_pressure_sensitivty: prom[2],
            temperature_coefficient_of_pressure_offset: prom[3],
            reference_temperature: prom[4],
            temperature_coefficient_of_temperature: prom[5],
        }
    }

    pub async fn read_pressure(&mut self, resolution: ms5837regs::Resolution) -> u32 {
        let mut data = [0u8; 4];
        let _ = self
            .driver
            .write_bytes(&[ms5837regs::READ_D1_BASE | resolution as u8])
            .await;
        Timer::after_micros(600).await;
        let _ = self
            .driver
            .write_read_bytes(&[ms5837regs::ADC_READ], &mut data[1..])
            .await;
        let d2 = u32::from_be_bytes(data);
        d2
    }

    pub async fn read_temperature(&mut self, resolution: ms5837regs::Resolution) -> u32 {
        let mut data = [0u8; 4];
        let _ = self
            .driver
            .write_bytes(&[ms5837regs::READ_D2_BASE | resolution as u8])
            .await;
        Timer::after_micros(600).await;
        let _ = self
            .driver
            .write_read_bytes(&[ms5837regs::ADC_READ], &mut data[1..])
            .await;

        let d2 = u32::from_be_bytes(data);
        d2
    }

    pub async fn altitude(&mut self) -> f32 {
        let ( pressure, temperature ) = self.calculate_temperature_pression(Osr256).await;
        (1.0 - powf(pressure/1013.25, 0.190284)) * 145366.45 * 0.3048
    }

    pub async fn depth(&mut self, density: f32) -> f32 {
        let ( pressure, temperature ) = self.calculate_temperature_pression(Osr256).await;
        ((pressure / 1013.25) - 1.0) * 10.0
    }



    pub async fn calculate_temperature_pression(
        &mut self,
        resolution: ms5837regs::Resolution,
    ) -> (f32, f32) {
        let d2 = self.read_temperature(resolution).await;
        let d1 = self.read_pressure(resolution).await;

        // info!("D1: {} D2: {}", d1, d2);

        let FactoryCalibrationData {
            pressure_sensitivity,
            pressure_offset,
            temperature_coefficient_of_pressure_sensitivty,
            temperature_coefficient_of_pressure_offset,
            reference_temperature,
            temperature_coefficient_of_temperature,
        } = self.calibration_data;

        let (
            pressure_sensitivity,
            pressure_offset,
            temperature_coefficient_of_pressure_sensitivty,
            temperature_coefficient_of_pressure_offset,
            reference_temperature,
            temperature_coefficient_of_temperature,
        ): (f32, f32, f32, f32, f32, f32) = (
            pressure_sensitivity.into(),
            pressure_offset.into(),
            temperature_coefficient_of_pressure_sensitivty.into(),
            temperature_coefficient_of_pressure_offset.into(),
            reference_temperature.into(),
            temperature_coefficient_of_temperature.into(),
        );

        let dt = d2 as f32 - (reference_temperature * 256.0) as f32;
        // info!("dt: {}", dt);
        // Actual temperature = 2000 + dT * temperature_sensitivity
        let temperature = 2000.0 + (dt * (temperature_coefficient_of_temperature / 8388608.0));

        // info!("Temperature: {}", temperature);

        let mut t2;
        let mut offset2;
        let mut sensitivity2;

        // Second order temperature compensation
        if temperature < 2000.0 {
            t2 = (3.0 * powf(dt, 2.0)) / 8589934592.0;
            offset2 = 3.0 * powf((temperature - 2000.0), 2.0) / 2.0;
            sensitivity2 = 5.0 * powf((temperature - 2000.0), 2.0) / 8.0;

            if temperature < -1500.0 {
                offset2 += 7.0 * powf((temperature + 1500.0),2.0);
                sensitivity2 += 4.0 * powf((temperature + 1500.0),2.0);
            }
        } else {
            t2 = (2.0 * powf(dt, 2.0)) / 137438953472.0;
            offset2 = powf((temperature - 2000.0),2.0) / 16.0;
            sensitivity2 = 0.0;
        }

        // info!("t2: {}, offset2: {}, sensitivity: {}", t2, offset2, sensitivity2);

        // OFF = OFF_T1 + TCO * dT
        let mut offset =
            (pressure_offset * 65536.0) + ((temperature_coefficient_of_pressure_offset * dt) / 128.0);
        offset -= offset2;

        // Sensitivity at actual temperature = SENS_T1 + TCS * dT
        let mut sensitivty = (pressure_sensitivity * 32768.0)
            + ((temperature_coefficient_of_pressure_sensitivty * dt) / 256.0);
        sensitivty -= sensitivity2;

        // Temperature compensated pressure = D1 * SENS - OFF
        let pressure = (((d1 as f32 * sensitivty) / 2097152.0) - offset) / 8192.0;
        let t3 = temperature - t2;


        (pressure as f32 / 10.0, t3 as f32 / 100.0)
    }
}

#[embassy_executor::task]
pub async fn ms5837_task(sensor: I2CDriver<'static>) {
    let mut ms5837_sensor = MS5837::new(sensor);
    ms5837_sensor.init().await;
    loop {
        let result = ms5837_sensor
            .calculate_temperature_pression(ms5837regs::Resolution::Osr256)
            .await;
        defmt::info!("Pressure: {} Temperature: {}", result.0, result.1);
        info!("Altitude: {}, Depth: {}", ms5837_sensor.altitude().await, ms5837_sensor.depth(1.0).await);
        // let result = ms5837_sensor.read_temperature(ms5837regs::Resolution::Osr256).await;
        // let normalised = ms5837_sensor.normalise_temperature(result);
        // defmt::info!("Pressure: {} Temperature: {}", result, normalised);
        //
        // let mut state = crate::STATE.lock().await;
        // state.pressure = result.0;
        // state.temperature = result.1;
        Timer::after_millis(1000).await;
        if (LOW_POWER_MODE.load(Ordering::Relaxed) == true) {
            info!("[MS5837] Low power mode");
            return;
        }
    }
}

use embassy_time::Timer;

use crate::i2cdriver::I2CDriver;

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
        PROM_READ_BASE: 0xA0,
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
    prom: [u16; 8],
}

impl<'a> MS5837<'a> {
    pub fn new(device: I2CDriver<'a>) -> Self {
        MS5837 { driver: device, prom: [0; 8] }
    }

    pub async fn init(&mut self) {
        self.driver.write_bytes(&[ms5837regs::RESET]).await;
        Timer::after_millis(10).await;
        self.read_prom().await;
    }

    pub async fn read_prom(&mut self) {
        // Read PROM coefficients
        for i in 0..8 {
            let mut data = [0; 2];
            self.driver.write_read(&[ms5837regs::PROM_READ_BASE + (i as u8 * 2)], &mut data);
            self.prom[i] = u16::from_be_bytes(data);
        }
    }

    fn crc4(n_prom: &mut [u16; 8]) -> u8 {
        let mut n_rem: u16 = 0; // Remainder
    
        // Mask the first element and set the last to 0
        n_prom[0] &= 0x0FFF;
        n_prom[7] = 0;
    
        // Main loop
        for cnt in 0..16 {
            if cnt % 2 == 1 {
                n_rem ^= (n_prom[cnt >> 1] & 0x00FF) as u16;
            } else {
                n_rem ^= (n_prom[cnt >> 1] >> 8) as u16;
            }
    
            // Perform operations on each bit
            for _ in 0..8 {
                if n_rem & 0x8000 != 0 {
                    n_rem = (n_rem << 1) ^ 0x3000;
                } else {
                    n_rem <<= 1;
                }
            }
        }
    
        // Final processing
        n_rem = (n_rem >> 12) & 0x000F;
        (n_rem ^ 0x00) as u8
    }
    

    async fn read_pressure(&mut self, resolution: ms5837regs::Resolution) -> u32 {
        let mut data = [0u8; 3];
        self.driver.write_read(&[ms5837regs::READ_D1_BASE + resolution as u8], &mut data).await;
        let d2 = u32::from_be_bytes([0, data[0], data[1], data[2]]);
        d2
    }

    async fn read_temperature(&mut self, resolution: ms5837regs::Resolution) -> u32 {
        let mut data = [0u8; 3];
        self.driver.write_read(&[ms5837regs::READ_D2_BASE + resolution as u8], &mut data).await;
        let d2 = u32::from_be_bytes([0, data[0], data[1], data[2]]);
        d2
    }

    pub async fn calculate_temperature_pression(&mut self, resolution: ms5837regs::Resolution) -> (f32, f32) {
        let d1 = self.read_pressure(resolution).await;
        let d2 = self.read_temperature(resolution).await;
        let c1 = self.prom[1] as i32; // Pressure sensitivity
        let c2 = self.prom[2] as i32; // Pressure offset
        let c3 = self.prom[3] as i32; // Temperature coefficient of pressure sensitivity
        let c4 = self.prom[4] as i32; // Temperature coefficient of pressure offset
        let c5 = self.prom[5] as i32; // Reference temperature
        let c6 = self.prom[6] as i32; // Temperature coefficient of temperature

        let d2 = d2 as i32;
        let d1 = d1 as i32;

        // First-order temperature and pressure calculations
        let dt = d2 - (c5 * 256);
        let temp = 2000 + ((dt * c6) / 8388608);

        let off = c2 * 65536 + (c4 * dt) / 128;
        let sens = c1 * 32768 + (c3 * dt) / 256;

        let pressure = ((d1 * sens) / 2097152 - off) / 8192;

        // Convert results to floating-point
        let temperature = temp as f32 / 100.0;
        let pressure = pressure as f32 / 10.0;

        (pressure, temperature)
    }
}

#[embassy_executor::task]
pub async fn ms5837_task(sensor: I2CDriver<'static>) {
    let mut ms5837_sensor = MS5837::new(sensor);
    ms5837_sensor.init().await;
    loop{
        let result = ms5837_sensor.calculate_temperature_pression(ms5837regs::Resolution::Osr256).await;
        defmt::info!("Pressure: {} Temperature: {}", result.0, result.1);
        Timer::after_millis(1000).await;
    }
}
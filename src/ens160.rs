use core::sync::atomic::Ordering;
use defmt::info;
use embassy_time::Timer;

#[allow(dead_code)]
mod ens160regs {
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
        PARTID: 0x00,
        OPMODE: 0x10,
        CONFIG: 0x11,
        COMMAND: 0x12,
        TEMPIN: 0x13,
        RHIN: 0x15,
        STATUS: 0x20,
        DATA_AQI: 0x21,
        DATA_TVOC: 0x22,
        DATA_ECO2: 0x24,
        DATA_T: 0x30,
        DATA_RH: 0x32,
        DATA_MISR: 0x38,
        GPRWRITE: 0x40,
        GPRREAD: 0x48
    }
}

pub struct ENS160<'a> {
    driver: I2CDriver<'a>,
}

impl<'a> ENS160<'a> {
    pub fn new(device: I2CDriver<'a>) -> Self {
        ENS160 { driver: device }
    }

    pub async fn init(&mut self) {
        let assigned_id = 0x0160;
        loop {
            let mut id: u16;
            let mut buffer: [u8; 2] = [0; 2];
            self.driver
                .write_read_bytes(&[ens160regs::PARTID], &mut buffer)
                .await;
            id = buffer[0] as u16 | (buffer[1] as u16) << 8;

            if assigned_id == id {
                defmt::info!("[ENS160] ID assigned = {}", id);
                return;
            } else {
                defmt::info!("[ENS160] Invalid id assigned = {}", id);
                Timer::after_millis(1000).await;
            }
        }
    }

    pub async fn set_opmode(&mut self, opmode: u8) {
        self.driver.write_bytes(&[ens160regs::OPMODE, opmode]).await;

        defmt::info!("[ENS160] Set opmode = {}", opmode);
    }

    pub async fn get_status(&mut self) -> [u8; 8] {
        let mut buffer: [u8; 8] = [0; 8];
        self.driver
            .write_read_bytes(&[ens160regs::STATUS], &mut buffer)
            .await;

        defmt::info!("[ENS160] Status = {}", buffer);

        buffer
    }

    pub async fn get_voc(&mut self) -> u16 {
        let mut voc: u16;
        let mut buffer: [u8; 2] = [0; 2];
        self.driver
            .write_read_bytes(&[ens160regs::DATA_TVOC], &mut buffer)
            .await;
        voc = buffer[0] as u16 | (buffer[1] as u16) << 8;
        voc
    }

    pub async fn get_eco2(&mut self) -> u16 {
        let mut voc: u16;
        let mut buffer: [u8; 2] = [0; 2];
        self.driver
            .write_read_bytes(&[ens160regs::DATA_ECO2], &mut buffer)
            .await;
        voc = buffer[0] as u16 | (buffer[1] as u16) << 8;
        voc
    }

    pub async fn get_aqi(&mut self) -> u8 {
        let mut buffer: [u8; 1] = [0; 1];
        self.driver
            .write_read_bytes(&[ens160regs::DATA_AQI], &mut buffer)
            .await;
        buffer[0]
    }
}

use crate::i2cdriver::I2CDriver;
use crate::LOW_POWER_MODE;

#[embassy_executor::task]
pub async fn ens_task(sensor: I2CDriver<'static>) {
    // Configure the LED pin as a push pull output and obtain handler.
    // On the Nucleo F091RC there's an on-board LED connected to pin PA5.
    let mut sensor = ENS160::new(sensor);
    sensor.init().await;
    sensor.set_opmode(0x02).await;
    loop {
        sensor.get_status().await;
        sensor.get_voc().await;
        sensor.get_eco2().await;
        sensor.get_aqi().await;
        Timer::after_millis(1000).await;
        if (LOW_POWER_MODE.load(Ordering::Relaxed) == true) {
            info!("[ENS160] Low power mode");
            return;
        }
    }
    // set_opmode(i2c, 0x53, 0x00);
}

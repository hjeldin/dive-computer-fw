use core::borrow::Borrow;

use embassy_stm32::{i2c::I2c, mode::Async};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};

pub struct I2CDriver<'a> {
    i2c: &'a Mutex<ThreadModeRawMutex, I2c<'static, Async>>,
    device_address: u8,
}

impl<'a> I2CDriver<'a> {
    pub fn new(i2c: &'a Mutex<ThreadModeRawMutex, I2c<'static, Async>>, device_address: u8) -> Self {
        Self { i2c, device_address }
    }

    // Example method to send a byte to the device
    pub async fn write_bytes(&self, byte: &[u8]) {
        let mut i2c = self.i2c.lock().await;
        i2c.write(self.device_address,byte).await.unwrap();
    }

    // Example method to read a byte from the device
    pub async fn read_byte(&self) -> Result<u8, I2cError> {
        let mut i2c = self.i2c.lock().await;
        let mut buffer = [0u8];
        i2c.read(self.device_address, &mut buffer).await.unwrap();
        Ok(buffer[0])
    }

    pub async fn write_read(&self, register: &[u8], buffer: &mut [u8]) {
        let mut i2c = self.i2c.lock().await;
        let res = i2c.blocking_write_read(self.device_address, register, buffer);
        if !res.is_ok() {   
            defmt::info!("err = {}", res.err());
        }
    }
}

#[derive(Debug)]
pub enum I2cError {
    WriteError,
    ReadError,
    Timeout,
}

impl From<embassy_stm32::i2c::Error> for I2cError {
    fn from(_error: embassy_stm32::i2c::Error) -> Self {
        I2cError::WriteError  // A basic conversion, modify as needed for your application
    }
}
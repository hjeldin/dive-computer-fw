use core::borrow::Borrow;

use embassy_stm32::{i2c::I2c, mode::Async};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embedded_hal_async::i2c::ErrorType;

pub struct I2CDriver<'a> {
    i2c: &'a Mutex<ThreadModeRawMutex, I2c<'static, Async>>,
    device_address: u8,
}

impl ErrorType for I2CDriver<'_> {
    type Error = embassy_stm32::i2c::Error;
}

impl embedded_hal_async::i2c::I2c<u8> for I2CDriver<'_> {
    async fn transaction(
        &mut self,
        _: u8,
        _: &mut [embedded_hal_async::i2c::Operation<'_>],
    ) -> Result<(), <Self as embedded_hal_async::i2c::ErrorType>::Error> {
        Ok(())
    }

    async fn read(&mut self, address: u8, read: &mut [u8]) -> Result<(), Self::Error> {
        self.read_byte().await.unwrap();
        Ok(())
    }

    async fn write(&mut self, address: u8, write: &[u8]) -> Result<(), Self::Error> {
        self.write_bytes(write).await.unwrap();
        Ok(())
    }

    async fn write_read(&mut self, address: u8, write: &[u8], read: &mut [u8]) -> Result<(), Self::Error> {
        let res = self.write_read_bytes(write, read).await;
        if res.is_err() {
            defmt::error!("Error reading/writing from I2C device {:x}", address);
            return Err(Self::Error::Timeout);
        }
        Ok(())
    }

}

impl<'a> I2CDriver<'a> {
    pub fn new(
        i2c: &'a Mutex<ThreadModeRawMutex, I2c<'static, Async>>,
        device_address: u8,
    ) -> Self {
        Self {
            i2c,
            device_address,
        }
    }

    pub async fn write_bytes(&self, byte: &[u8]) -> Result<(), I2cError> {
        let mut i2c = self.i2c.lock().await;
        let res = i2c.write(self.device_address, byte).await;
        match res {
            Ok(_) => Ok(()),
            Err(_) => {
                defmt::error!("Error writing to I2C device {:x}", self.device_address);
                return Err(I2cError::WriteError);
            }
        }
    }

    pub async fn read_byte(&self) -> Result<u8, I2cError> {
        let mut i2c = self.i2c.lock().await;
        let mut buffer = [0u8];
        let res = i2c.read(self.device_address, &mut buffer).await;
        match res {
            Ok(_) => Ok(buffer[0]),
            Err(_) => {
                defmt::error!("Error reading from I2C device {:x}", self.device_address);
                return Err(I2cError::ReadError);
            }
        }
    }

    pub async fn write_read_bytes(
        &self,
        register: &[u8],
        buffer: &'a mut [u8],
    ) -> Result<&'a mut [u8], I2cError> {
        let mut i2c = self.i2c.lock().await;
        let res = i2c.write_read(self.device_address, register, buffer).await;
        match res {
            Ok(_) => Ok(buffer),
            Err(_) => {
                defmt::error!(
                    "Error reading/writing from I2C device {:x}",
                    self.device_address
                );
                return Err(I2cError::ReadError);
            }
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
        I2cError::WriteError // A basic conversion, modify as needed for your application
    }
}

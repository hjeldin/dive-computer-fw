use core::convert::TryInto;

use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    mode::{Async, Blocking},
    pac::metadata::Peripheral,
    spi::Spi,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::Timer;

pub struct SPIDriver<'a> {
    spi: &'a Mutex<ThreadModeRawMutex, Spi<'static, Async>>,
    dc: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
    rst: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
}

impl<'a> SPIDriver<'a> {
    pub fn new(
        spi: &'a Mutex<ThreadModeRawMutex, Spi<'static, Async>>,
        dc: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
        rst: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
    ) -> Self {
        Self { spi, dc, rst }
    }

    pub async fn reset(&self) {
        let mut rst = self.rst.try_lock().unwrap();
        rst.set_high();
        Timer::after_millis(200).await;
        rst.set_low();
        Timer::after_millis(200).await;
        rst.set_high();
        Timer::after_millis(200).await;
    }

    pub async fn soft_reset(&self) {
        self.reset().await;
        self.write_command(&[0x01]);
        Timer::after_millis(200).await;
    }

    pub fn sleep(&self) {
        self.write_command(&[0x10]);
    }

    pub fn enable_write_data(&self) {
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_high();
    }

    pub fn write_command(&self, byte: &[u8]) {
        let mut spi = self.spi.try_lock().unwrap();
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_low();
        spi.blocking_write(byte).unwrap();
        dc.set_high();
    }

    pub fn write_data(&self, byte: &[u8]) {
        let mut spi = self.spi.try_lock().unwrap();
        defmt::info!("write_data {}", byte.len());
        spi.blocking_write(byte).unwrap()
    }

    pub fn write_data_continue(&self, byte: &[u8]) {
        let mut spi = self.spi.try_lock().unwrap();
        defmt::info!("write_data {}", byte.len());
        spi.blocking_write(byte).unwrap()
    }

    pub fn read_bytes(&self, buffer: &mut [u8]) {
        let mut spi = self.spi.try_lock().unwrap();
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_high();
        spi.blocking_read(buffer).unwrap();
        dc.set_low();
    }
}

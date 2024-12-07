use embassy_stm32::{
    gpio::{AnyPin, Level, Output, Speed},
    mode::Blocking,
    pac::metadata::Peripheral,
    spi::Spi,
};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::Timer;

pub struct SPIDriver<'a> {
    spi: &'a Mutex<ThreadModeRawMutex, Spi<'static, Blocking>>,
    dc: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
    rst: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
}

impl<'a> SPIDriver<'a> {
    pub fn new(
        spi: &'a Mutex<ThreadModeRawMutex, Spi<'static, Blocking>>,
        dc: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
        rst: &'a Mutex<ThreadModeRawMutex, Output<'static>>,
    ) -> Self {
        Self { spi, dc, rst }
    }

    pub async fn reset(&self) {
        let mut rst = self.rst.lock().await;
        rst.set_high();
        Timer::after_millis(200).await;
        rst.set_low();
        Timer::after_millis(200).await;
        rst.set_high();
        Timer::after_millis(200).await;
    }

    pub async fn sleep(&self) {
        self.write_command(&[0x10]).await;
    }

    pub async fn write_command(&self, byte: &[u8]) {
        let mut spi = self.spi.lock().await;
        let mut dc = self.dc.lock().await;
        dc.set_low();
        spi.blocking_write(byte).unwrap()
    }

    pub async fn write_data(&self, byte: &[u8]) {
        let mut spi = self.spi.lock().await;
        let mut dc = self.dc.lock().await;
        dc.set_high();
        spi.blocking_write(byte).unwrap()
    }

    pub async fn read_bytes(&self, buffer: &mut [u8]) {
        let mut spi = self.spi.lock().await;
        let mut dc = self.dc.lock().await;
        dc.set_high();
        spi.blocking_read(buffer).unwrap()
    }
}

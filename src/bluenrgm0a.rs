use embassy_stm32::gpio::Output;
use embassy_stm32::mode::Async;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_time::Timer;

pub struct BLUENRGM0A {
    driver: embassy_stm32::spi::Spi<'static, Async>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
}

impl BLUENRGM0A {
    pub fn new(
        device: embassy_stm32::spi::Spi<'static, Async>,
        rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    ) -> Self {
        BLUENRGM0A {
            driver: device,
            rst,
        }
    }
}

#[embassy_executor::task]
pub async fn screen_task(
    nrg_spi: embassy_stm32::spi::Spi<'static, Async>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
) {
    let mut bluenrg = BLUENRGM0A::new(nrg_spi, rst);
    // bluenrg.rst.lock().await.set_high().unwrap();
    // bluenrg.driver.write(&[0x00]).await.unwrap();
    // bluenrg.rst.lock().await.set_low().unwrap();
    // bluenrg.driver.write(&[0x01]).await.unwrap();
    loop {
        bluenrg.driver.blocking_write(&[0x00u8]);
        Timer::after_millis(1000).await;
        bluenrg.driver.blocking_write(&[0x01u8]);
        Timer::after_millis(1000).await;
    }
}
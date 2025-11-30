use core::ops::Deref;
use embassy_stm32::peripherals::SDMMC1;
use defmt::{info, unwrap, Debug2Format};
use embassy_stm32::sdmmc::{DataBlock, Sdmmc};
use embassy_stm32::time::Hertz;
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::rwlock::RwLock;
use embassy_time::{Duration, Timer, WithTimeout};
use embedded_sdmmc::asynchronous::{Block, BlockCount, BlockDevice, BlockIdx, Mode, VolumeIdx, VolumeManager};
use crate::state;

pub struct TimeSource;

impl embedded_sdmmc::asynchronous::TimeSource for TimeSource {
    fn get_timestamp(&self) -> embedded_sdmmc::asynchronous::Timestamp {
        embedded_sdmmc::asynchronous::Timestamp {
            year_since_1970: 0,
            zero_indexed_month: 0,
            zero_indexed_day: 0,
            hours: 0,
            minutes: 0,
            seconds: 0,
        }
    }
}

struct SDMMCDevice<'a> {
    sdmmc: &'a Mutex<ThreadModeRawMutex, Sdmmc<'a, SDMMC1>>
}

impl<'a> SDMMCDevice<'a> {
    fn new(sdmmc: &'a Mutex<ThreadModeRawMutex, Sdmmc<'a, SDMMC1>>) -> Self {
        Self { sdmmc }
    }

    async fn init(&mut self) {
        // Initialize the SDMMC device
        // This is where you would set up the device, if needed

        // Should print 400kHz for initialization
        let mut sdmmc = self.sdmmc.lock().await;
        info!("Configured clock: {}", sdmmc.clock().0);

        let mut _err = None;

        loop {
            match sdmmc.init_sd_card(Hertz(24_000_000)).await {
                Ok(_) => {
                    info!("Card initialized");
                    break;
                },
                Err(e) => {
                    // info!("waiting for card error, retrying: {:?}", e);
                    _err = Some(e);
                }
            }
            Timer::after(Duration::from_millis(1000)).await;
        }

        let card = unwrap!(sdmmc.card());

        info!("Card: {:#?}", Debug2Format(card));
        info!("Clock: {}", sdmmc.clock());
    }
}

impl<'a> BlockDevice for SDMMCDevice<'a> {
    type Error = ();

    async fn read(&self, blocks: &mut [Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        // Read blocks from the SD card
        let mut sdmmc = critical_section::with(|_cs| {
            self.sdmmc.try_lock().unwrap()
        });
        Ok(for (i, block) in blocks.iter_mut().enumerate() {
            let block_idx = start_block_idx.0 + i as u32;
            let mut datablock = DataBlock([0u8; 512]);
            sdmmc.read_block(block_idx, &mut datablock).with_timeout(Duration::from_millis(100)).await.unwrap().expect("TODO: panic message");
            block.contents = datablock.0;
        })
    }

    async fn write(&self, blocks: &[Block], start_block_idx: BlockIdx) -> Result<(), Self::Error> {
        let mut sdmmc = critical_section::with(|_cs| {
            self.sdmmc.try_lock().unwrap()
        });
        Ok(for (i, block) in blocks.iter().enumerate() {
            let block_idx = start_block_idx.0 + i as u32;
            let mut datablock = DataBlock([0u8; 512]);
            datablock.0.copy_from_slice(&block.contents);
            sdmmc.write_block(block_idx, &datablock).await.unwrap();
        })
    }

    async fn num_blocks(&self) -> Result<BlockCount, Self::Error> {
        // Return the number of blocks on the SD card
        let _sdmmc = critical_section::with(|_cs| {
            self.sdmmc.try_lock().unwrap()
        });
        // let mut card = sdmmc.card().unwrap().get_sd_card();
        Ok(BlockCount(512))
    }
}

#[embassy_executor::task]
pub async fn sd_card_task(sdmmc: &'static Mutex<ThreadModeRawMutex, Sdmmc<'static, SDMMC1>>, state: &'static RwLock<ThreadModeRawMutex, state::State>) {
    let mut device = SDMMCDevice::new(sdmmc);
    device.init().await;

    let volume_manager = VolumeManager::new(device, TimeSource);

    let volume0 = volume_manager.open_volume(VolumeIdx(0)).await.unwrap();

    info!("Volume 0: {:#?}", Debug2Format(&volume0));

    let root_dir = volume0.open_root_dir().unwrap();
    info!("Root dir: {:#?}", Debug2Format(&root_dir));


    let mut file = root_dir.open_file_in_dir("test.txt", Mode::ReadWriteCreateOrAppend).await.unwrap();

    loop {
        Timer::after_millis(1000).await;
        let state = state.read().await;
        let mut buf = [0u8; 64];
        let text = format_no_std::show(&mut buf, format_args!("{:.5}\n{:.1}\n", state.pressure, state.temperature)).unwrap();
        file.write(&text.as_bytes()).await.unwrap();
        let accel = format_no_std::show(&mut buf, format_args!("Accel(m/s²): {:.2}, {:.2}, {:.2}\n", state.accel_x, state.accel_y, state.accel_z)).unwrap();
        file.write(&accel.as_bytes()).await.unwrap();
        let gyro = format_no_std::show(&mut buf, format_args!("Gyro(rad/s): {:.2}, {:.2}, {:.2}\n", state.gyro_x, state.gyro_y, state.gyro_z)).unwrap();
        file.write(&gyro.as_bytes()).await.unwrap();
        file.flush().await.unwrap();

        // TODO: find a way to catch communication error with the SD card
    }
}

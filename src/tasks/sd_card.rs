use chrono::NaiveDateTime;
use defmt::{info, unwrap, Debug2Format};
use embassy_stm32::peripherals::{DMA2_CH4, SDMMC1, PC10, PC11, PC12, PC8, PC9, PD2};
use embassy_stm32::sdmmc::{DataBlock, Sdmmc};
use embassy_stm32::time::Hertz;
use embassy_time::Timer;
use crate::Irqs;
// use fatfs_embedded;
// use fatfs_embedded::fatfs::diskio::{DiskResult, IoctlCommand};
//
// struct SDDriver {
//
// }
//
// impl fatfs_embedded::fatfs::diskio::FatFsDriver for SDDriver {
//     fn disk_status(&self, drive: u8) -> u8 {
//         todo!()
//     }
//
//     fn disk_initialize(&mut self, drive: u8) -> u8 {
//         todo!()
//     }
//
//     fn disk_read(&mut self, drive: u8, buffer: &mut [u8], sector: u32) -> DiskResult {
//         todo!()
//     }
//
//     fn disk_write(&mut self, drive: u8, buffer: &[u8], sector: u32) -> DiskResult {
//         todo!()
//     }
//
//     fn disk_ioctl(&self, data: &mut IoctlCommand) -> DiskResult {
//         todo!()
//     }
//
//     fn get_fattime(&self) -> NaiveDateTime {
//         todo!()
//     }
// }

#[embassy_executor::task]
pub async fn sd_card_task(sdmmc: SDMMC1, dma2_ch4: DMA2_CH4, pc12: PC12, pd2: PD2, pc8: PC8, pc9: PC9, pc10: PC10, pc11: PC11) {
    let mut sdmmc = Sdmmc::new_4bit(
        sdmmc,
        Irqs,
        dma2_ch4,
        pc12,
        pd2,
        pc8,
        pc9,
        pc10,
        pc11,
        Default::default(),
    );

    // Should print 400kHz for initialization
    info!("Configured clock: {}", sdmmc.clock().0);

    let mut err = None;
    loop {
        match sdmmc.init_card(Hertz(48_000_000)).await {
            Ok(_) => {
                info!("Card initialized");
                break;
            },
            Err(e) => {
                // if err != Some(e) {
                    info!("waiting for card error, retrying: {:?}", e);
                    err = Some(e);
                // }
            }
        }
        Timer::after_millis(100).await;
    }

    let card = unwrap!(sdmmc.card());

    info!("Card: {:#?}", Debug2Format(card));
    info!("Clock: {}", sdmmc.clock());

    // Arbitrary block index
    let block_idx = 16;

    // SDMMC uses `DataBlock` instead of `&[u8]` to ensure 4 byte alignment required by the hardware.
    let mut block = DataBlock([0u8; 512]);

    sdmmc.read_block(block_idx, &mut block).await.unwrap();
    info!("Read: {=[u8]:X}...{=[u8]:X}", block[..8], block[512 - 8..]);

    // if !ALLOW_WRITES {
    //     info!("Writing is disabled.");
    //     loop {}
    // }

    info!("Filling block with 0x55");
    block.fill(0x55);
    sdmmc.write_block(block_idx, &block).await.unwrap();
    info!("Write done");

    sdmmc.read_block(block_idx, &mut block).await.unwrap();
    info!("Read: {=[u8]:X}...{=[u8]:X}", block[..8], block[512 - 8..]);

    info!("Filling block with 0xAA");
    block.fill(0xAA);
    sdmmc.write_block(block_idx, &block).await.unwrap();
    info!("Write done");

    sdmmc.read_block(block_idx, &mut block).await.unwrap();
    info!("Read: {=[u8]:X}...{=[u8]:X}", block[..8], block[512 - 8..]);

    loop {
        Timer::after_millis(1000).await;
    }
}

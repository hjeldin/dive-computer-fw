use defmt::info;
use embassy_stm32::dma::Transfer;
use embassy_stm32::peripherals::{PA6, PA7, PB0, PB1, PB10, PB11, QUADSPI};
use embassy_stm32::qspi::enums::{AddressSize, ChipSelectHighTime, FIFOThresholdLevel, MemorySize, QspiWidth, SampleShifting};
use embassy_stm32::qspi::{Config, Qspi, TransferConfig};
use embassy_stm32::rtc::Rtc;
use embassy_stm32::Peri;
use embassy_time::Timer;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
use static_cell::StaticCell;

// QUADSPI flash address for storing timestamp
// QUADSPI flash is memory-mapped at 0x90000000
// Winbond W25W256JVFIQ: 256Mbit (32MB) = 0x2000000 bytes
// Using last 4KB sector: 32MB - 4KB = 0x01FFC000
const TIMESTAMP_FLASH_OFFSET: u32 = 0x01FFC000; // Last 4KB sector of 32MB flash
const TIMESTAMP_FLASH_ADDR: u32 = 0x90000000 + TIMESTAMP_FLASH_OFFSET;

// Timestamp data structure: Unix timestamp as u64 (8 bytes)
// W25W256JVFIQ specifications:
// - 4KB sectors (erase unit)
// - 256-byte pages (program unit)
const FLASH_WRITE_SIZE: usize = 256;
const FLASH_SECTOR_SIZE: u32 = 4096; // 4KB sectors

#[embassy_executor::task]
pub async fn flash_timestamp_task(
    qspi: Peri<'static, QUADSPI>,
    io0: Peri<'static, PB1>,
    io1: Peri<'static, PB0>,
    io2: Peri<'static, PA7>,
    io3: Peri<'static, PA6>,
    sck: Peri<'static, PB10>,
    cs: Peri<'static, PB11>,
    rtc: &'static StaticCell<Rtc>,
) {
    let mut config = Config::default();
    config.memory_size = MemorySize::_32MiB;
    config.address_size = AddressSize::_24bit;
    config.prescaler = 16;
    config.cs_high_time = ChipSelectHighTime::_5Cycle;
    config.fifo_threshold = FIFOThresholdLevel::_17Bytes;
    config.sample_shifting = SampleShifting::None;
    
    let mut qspi_flash = Qspi::new_blocking_bank1(
        qspi,
        io0,
        io1,
        io2,
        io3,
        sck,
        cs,
        config,
    );

    return;

    loop {
        let mut buff = [0u8; 256];

        let mut transfer_config = TransferConfig::default();
        transfer_config.address = Some(TIMESTAMP_FLASH_ADDR);
        transfer_config.awidth = QspiWidth::QUAD;
        transfer_config.dwidth = QspiWidth::SING;
        transfer_config.instruction = 0x03;
        transfer_config.iwidth = QspiWidth::SING;

        // qspi_flash.blocking_read(&mut buff, transfer_config);

        info!("Timestamp: {:?}", buff);
        Timer::after_secs(60).await; // Write timestamp every 60 seconds
    }
}
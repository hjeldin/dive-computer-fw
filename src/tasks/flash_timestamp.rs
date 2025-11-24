// use defmt::info;
// use embassy_stm32::peripherals::{PA6, PA7, PB0, PB1, PB10, PB11, QUADSPI};
// use embassy_stm32::qspi::{Qspi, Config};
// use embassy_stm32::rtc::Rtc;
// use embassy_stm32::Peri;
// use embassy_time::Timer;
// use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};
// use static_cell::StaticCell;
//
// // QUADSPI flash address for storing timestamp
// // QUADSPI flash is memory-mapped at 0x90000000
// // Winbond W25W256JVFIQ: 256Mbit (32MB) = 0x2000000 bytes
// // Using last 4KB sector: 32MB - 4KB = 0x01FFC000
// const TIMESTAMP_FLASH_OFFSET: u32 = 0x01FFC000; // Last 4KB sector of 32MB flash
// const TIMESTAMP_FLASH_ADDR: u32 = 0x90000000 + TIMESTAMP_FLASH_OFFSET;
//
// // Timestamp data structure: Unix timestamp as u64 (8 bytes)
// // W25W256JVFIQ specifications:
// // - 4KB sectors (erase unit)
// // - 256-byte pages (program unit)
// const FLASH_WRITE_SIZE: usize = 256;
// const FLASH_SECTOR_SIZE: u32 = 4096; // 4KB sectors
//
// #[embassy_executor::task]
// pub async fn flash_timestamp_task(
//     qspi: Peri<'static, QUADSPI>,
//     io0: Peri<'static, PB1>,
//     io1: Peri<'static, PB0>,
//     io2: Peri<'static, PA7>,
//     io3: Peri<'static, PA6>,
//     sck: Peri<'static, PB10>,
//     cs: Peri<'static, PB11>,
//     rtc: &'static StaticCell<Rtc>,
// ) {
//     // Initialize QUADSPI with configuration for Winbond W25W256JVFIQ
//     // Specifications: 256Mbit (32MB), 4KB sectors, 256-byte pages
//     // Supports Quad-SPI up to 133MHz
//     let mut config = Config::default();
//     // Configure flash size: 32MB = 0x2000000 bytes = 25 address bits
//     // Flash size is specified as log2(size) - 1, so for 32MB: log2(32MB) - 1 = 25 - 1 = 24
//     // But embassy-stm32 might use a different encoding, check Config fields
//     // For now, using default config which should work, but may need adjustment
//     let mut qspi_flash = Qspi::new_blocking_bank1(
//         qspi,
//         io0,
//         io1,
//         io2,
//         io3,
//         sck,
//         cs,
//         config,
//     );
//
//     loop {
//         Timer::after_secs(60).await; // Write timestamp every 60 seconds
//
//         // Get current datetime from RTC
//         let rtc_ref = rtc.get();
//         let datetime = rtc_ref.now();
//
//         // Convert DateTime to Unix timestamp (seconds since 1970-01-01)
//         let unix_timestamp = datetime_to_unix_timestamp(&datetime);
//
//         info!("Writing timestamp to QUADSPI flash: {} ({} seconds since epoch)",
//               unix_timestamp, unix_timestamp);
//
//         // Prepare data buffer aligned to flash write size
//         let mut write_buffer = [0xFFu8; FLASH_WRITE_SIZE]; // Flash defaults to 0xFF
//
//         // Write timestamp as u64 (little-endian) at the start of the buffer
//         let timestamp_bytes = unix_timestamp.to_le_bytes();
//         write_buffer[0..8].copy_from_slice(&timestamp_bytes);
//
//         // Erase the sector before writing (flash requires erase before write)
//         // W25W256JVFIQ uses 4KB sectors
//         let sector_addr = TIMESTAMP_FLASH_OFFSET & !(FLASH_SECTOR_SIZE - 1); // Align to 4KB boundary
//
//         // Erase the flash sector (4KB)
//         match qspi_flash.erase(sector_addr, sector_addr + FLASH_SECTOR_SIZE) {
//             Ok(_) => {
//                 info!("QUADSPI flash sector erased successfully");
//             }
//             Err(e) => {
//                 info!("Failed to erase QUADSPI flash sector: {:?}", e);
//                 continue;
//             }
//         }
//
//         // Write the timestamp data
//         match qspi_flash.write(TIMESTAMP_FLASH_OFFSET, &write_buffer) {
//             Ok(_) => {
//                 info!("Timestamp written to QUADSPI flash successfully");
//             }
//             Err(e) => {
//                 info!("Failed to write timestamp to QUADSPI flash: {:?}", e);
//             }
//         }
//     }
// }
//
// fn datetime_to_unix_timestamp(dt: &embassy_stm32::rtc::DateTime) -> u64 {
//     // Convert DateTime to Unix timestamp (seconds since 1970-01-01)
//     // DateTime structure in embassy-stm32 - accessing fields via Debug or methods
//     // Using format! macro to extract values if fields are private
//     use core::fmt::Write;
//
//     // Try to format DateTime and extract components
//     // If DateTime has public fields, access them directly
//     // Otherwise, use a workaround
//
//     // embassy-stm32 DateTime may have public fields or accessor methods
//     // For now, using a pattern match or field access
//     // Check embassy-stm32 docs for the correct way to access DateTime fields
//
//     // Simple conversion - assuming fields are accessible
//     // If compilation fails, check embassy-stm32 DateTime API documentation
//     let year = dt.year() as u64;
//     let month = dt.month() as u64;
//     let day = dt.day() as u64;
//     let hour = dt.hour() as u64;
//     let minute = dt.minute() as u64;
//     let second = dt.second() as u64;
//
//     // Days since 1970-01-01 (simplified calculation)
//     let mut days = (year - 1970) * 365;
//     days += (year - 1969) / 4; // Approximate leap years
//
//     // Add days for months (simplified)
//     let month_days = [0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334];
//     if month > 1 && month <= 12 {
//         days += month_days[(month - 1) as usize];
//     }
//     days += day - 1;
//
//     // Convert to seconds
//     let seconds = days * 86400 + hour * 3600 + minute * 60 + second;
//
//     seconds
// }
//

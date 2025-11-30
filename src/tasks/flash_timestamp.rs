use defmt::{info, warn, error};
use embassy_stm32::peripherals::{PA6, PA7, PB0, PB1, PB10, PB11, QUADSPI};
use embassy_stm32::qspi::enums::{AddressSize, ChipSelectHighTime, DummyCycles, FIFOThresholdLevel, MemorySize, QspiWidth, SampleShifting};
use embassy_stm32::qspi::{Config, Qspi, TransferConfig};
use embassy_stm32::peripherals::RTC;
use embassy_stm32::rtc::{Rtc, RtcConfig, RtcTimeProvider, RtcError, DateTime, DayOfWeek};
use embassy_stm32::Peri;
use embassy_time::Timer;

// QUADSPI flash address for storing timestamp
// QUADSPI flash is memory-mapped at 0x90000000
// Winbond W25Q256JV: 256Mbit (32MB) = 0x2000000 bytes
// Using last 4KB sector: 32MB - 4KB = 0x01FFC000
const TIMESTAMP_FLASH_OFFSET: u32 = 0x01FFC000; // Last 4KB sector of 32MB flash

// Timestamp data structure: year (u16), month (u8), day (u8), hour (u8), minute (u8), second (u8) = 8 bytes
// Layout: [year_low, year_high, month, day, hour, minute, second, reserved]
// W25Q256JV specifications:
// - 4KB sectors (erase unit)
// - 256-byte pages (program unit)
const FLASH_PAGE_SIZE: usize = 256;
const FLASH_SECTOR_SIZE: u32 = 4096; // 4KB sectors
const DATETIME_DATA_SIZE: usize = 8; // 8 bytes for datetime data

// W25Q256JV Flash Commands
const CMD_READ: u8 = 0x03;                    // Standard Read
const CMD_FAST_READ: u8 = 0x0B;              // Fast Read
const CMD_QUAD_IO_FAST_READ: u8 = 0x6B;      // Quad I/O Fast Read
const CMD_PAGE_PROGRAM: u8 = 0x02;           // Page Program
const CMD_QUAD_PAGE_PROGRAM: u8 = 0x32;      // Quad Page Program
const CMD_SECTOR_ERASE: u8 = 0x20;           // 4KB Sector Erase
const CMD_BLOCK_ERASE_32K: u8 = 0x52;        // 32KB Block Erase
const CMD_BLOCK_ERASE_64K: u8 = 0xD8;        // 64KB Block Erase
const CMD_CHIP_ERASE: u8 = 0xC7;             // Chip Erase
const CMD_WRITE_ENABLE: u8 = 0x06;           // Write Enable
const CMD_WRITE_DISABLE: u8 = 0x04;          // Write Disable
const CMD_READ_STATUS_REG1: u8 = 0x05;       // Read Status Register 1
const CMD_READ_STATUS_REG2: u8 = 0x35;       // Read Status Register 2
const CMD_WRITE_STATUS_REG: u8 = 0x01;       // Write Status Register (SR1 and SR2)
const CMD_WRITE_STATUS_REG2: u8 = 0x31;      // Write Status Register 2 only
const CMD_READ_ID: u8 = 0x9F;                 // Read JEDEC ID
const CMD_ENABLE_RESET: u8 = 0x66;            // Enable Reset
const CMD_RESET: u8 = 0x99;                  // Reset Device

// Status Register bits
const SR1_BUSY: u8 = 0x01;                   // Write in progress
const SR1_WEL: u8 = 0x02;                    // Write enable latch
const SR2_QE: u8 = 0x02;                     // Quad Enable (bit 1)

/// W25Q256JV Flash Memory Driver
pub struct W25Q256JV {
    qspi: Qspi<'static, QUADSPI, embassy_stm32::mode::Blocking>,
}

impl W25Q256JV {
    /// Create a new W25Q256JV flash driver
    pub fn new(qspi: Qspi<'static, QUADSPI, embassy_stm32::mode::Blocking>) -> Self {
        let mut flash = Self { qspi };
        
        // Initialize flash: reset and enable quad mode
        flash.reset();
        flash.enable_quad_mode();
        
        flash
    }

    /// Execute a command without data
    fn exec_command(&mut self, cmd: u8) {
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::NONE,
            instruction: cmd,
            address: None,
            dummy: DummyCycles::_0,
        };
        self.qspi.blocking_command(transaction);
    }

    /// Read a register (Status Register 1 or 2)
    fn read_register(&mut self, cmd: u8) -> u8 {
        let mut buffer = [0u8; 1];
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: cmd,
            address: None,
            dummy: DummyCycles::_0,
        };
        self.qspi.blocking_read(&mut buffer, transaction);
        buffer[0]
    }

    /// Write a register
    fn write_register(&mut self, cmd: u8, value: u8) {
        let buffer = [value];
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: cmd,
            address: None,
            dummy: DummyCycles::_0,
        };
        self.enable_write();
        self.qspi.blocking_write(&buffer, transaction);
        self.wait_ready();
    }

    /// Read Status Register 1
    pub fn read_status_reg1(&mut self) -> u8 {
        self.read_register(CMD_READ_STATUS_REG1)
    }

    /// Read Status Register 2
    pub fn read_status_reg2(&mut self) -> u8 {
        self.read_register(CMD_READ_STATUS_REG2)
    }

    /// Check if flash is busy (write/erase in progress)
    pub fn is_busy(&mut self) -> bool {
        (self.read_status_reg1() & SR1_BUSY) != 0
    }

    /// Wait for flash to be ready (not busy)
    pub fn wait_ready(&mut self) {
        while self.is_busy() {
            // Busy wait
        }
    }

    /// Enable write operations
    pub fn enable_write(&mut self) {
        self.exec_command(CMD_WRITE_ENABLE);
        // Small delay to ensure command is processed
        // Note: In a real implementation, you might want to verify WEL bit
    }

    /// Disable write operations
    pub fn disable_write(&mut self) {
        self.exec_command(CMD_WRITE_DISABLE);
    }

    /// Enable Quad mode by setting QE bit in Status Register 2
    pub fn enable_quad_mode(&mut self) {
        let sr2 = self.read_status_reg2();
        if (sr2 & SR2_QE) == 0 {
            // QE bit not set, enable it using Write Status Register 2 command
            let buffer = [sr2 | SR2_QE];
            let transaction = TransferConfig {
                iwidth: QspiWidth::SING,
                awidth: QspiWidth::NONE,
                dwidth: QspiWidth::SING,
                instruction: CMD_WRITE_STATUS_REG2,
                address: None,
                dummy: DummyCycles::_0,
            };
            self.enable_write();
            self.qspi.blocking_write(&buffer, transaction);
            self.wait_ready();
            info!("Quad mode enabled");
        } else {
            info!("Quad mode already enabled");
        }
    }

    /// Reset the flash device
    pub fn reset(&mut self) {
        self.exec_command(CMD_ENABLE_RESET);
        self.exec_command(CMD_RESET);
        self.wait_ready();
        info!("Flash reset complete");
    }

    /// Read JEDEC ID (3 bytes: Manufacturer ID, Memory Type, Capacity)
    pub fn read_id(&mut self) -> [u8; 3] {
        let mut buffer = [0u8; 3];
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::NONE,
            dwidth: QspiWidth::SING,
            instruction: CMD_READ_ID,
            address: None,
            dummy: DummyCycles::_0,
        };
        self.qspi.blocking_read(&mut buffer, transaction);
        buffer
    }

    /// Read data from flash using Quad I/O Fast Read (recommended for W25Q256JV)
    pub fn read(&mut self, address: u32, buffer: &mut [u8]) {
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_QUAD_IO_FAST_READ,
            address: Some(address),
            dummy: DummyCycles::_8, // 8 dummy cycles for Quad I/O Fast Read
        };
        self.qspi.blocking_read(buffer, transaction);
    }

    /// Read data from flash using standard read (slower, but simpler)
    pub fn read_standard(&mut self, address: u32, buffer: &mut [u8]) {
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::SING,
            instruction: CMD_READ,
            address: Some(address),
            dummy: DummyCycles::_0,
        };
        self.qspi.blocking_read(buffer, transaction);
    }

    /// Write a single page (256 bytes) to flash
    /// Note: address must be page-aligned (address % 256 == 0)
    /// and data length must not exceed page boundary
    fn write_page(&mut self, address: u32, data: &[u8]) {
        assert!(
            data.len() <= FLASH_PAGE_SIZE,
            "Data length exceeds page size"
        );
        assert!(
            (address as usize % FLASH_PAGE_SIZE) + data.len() <= FLASH_PAGE_SIZE,
            "Write crosses page boundary"
        );

        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_QUAD_PAGE_PROGRAM,
            address: Some(address),
            dummy: DummyCycles::_0,
        };
        
        self.enable_write();
        self.qspi.blocking_write(data, transaction);
        self.wait_ready();
    }

    /// Write data to flash (handles page boundaries automatically)
    pub fn write(&mut self, address: u32, data: &[u8]) {
        let mut offset = 0;
        let mut current_addr = address;

        while offset < data.len() {
            // Calculate how much we can write in this page
            let page_offset = (current_addr as usize) % FLASH_PAGE_SIZE;
            let remaining_in_page = FLASH_PAGE_SIZE - page_offset;
            let remaining_data = data.len() - offset;
            let write_len = remaining_in_page.min(remaining_data);

            let chunk = &data[offset..offset + write_len];
            self.write_page(current_addr, chunk);

            offset += write_len;
            current_addr += write_len as u32;
        }
    }

    /// Erase a 4KB sector
    pub fn erase_sector(&mut self, address: u32) {
        assert!(
            address % FLASH_SECTOR_SIZE == 0,
            "Address must be sector-aligned"
        );

        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::NONE,
            instruction: CMD_SECTOR_ERASE,
            address: Some(address),
            dummy: DummyCycles::_0,
        };

        self.enable_write();
        self.qspi.blocking_command(transaction);
        self.wait_ready();
    }

    /// Erase a 32KB block
    pub fn erase_block_32k(&mut self, address: u32) {
        assert!(
            address % 32768 == 0,
            "Address must be 32KB-aligned"
        );

        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::NONE,
            instruction: CMD_BLOCK_ERASE_32K,
            address: Some(address),
            dummy: DummyCycles::_0,
        };

        self.enable_write();
        self.qspi.blocking_command(transaction);
        self.wait_ready();
    }

    /// Erase a 64KB block
    pub fn erase_block_64k(&mut self, address: u32) {
        assert!(
            address % 65536 == 0,
            "Address must be 64KB-aligned"
        );

        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::NONE,
            instruction: CMD_BLOCK_ERASE_64K,
            address: Some(address),
            dummy: DummyCycles::_0,
        };

        self.enable_write();
        self.qspi.blocking_command(transaction);
        self.wait_ready();
    }

    /// Erase entire chip (use with caution!)
    pub fn erase_chip(&mut self) {
        self.enable_write();
        self.exec_command(CMD_CHIP_ERASE);
        self.wait_ready();
        warn!("Chip erase completed");
    }
}

#[embassy_executor::task]
pub async fn flash_timestamp_task(
    qspi: Peri<'static, QUADSPI>,
    io0: Peri<'static, PB1>,
    io1: Peri<'static, PB0>,
    io2: Peri<'static, PA7>,
    io3: Peri<'static, PA6>,
    sck: Peri<'static, PB10>,
    cs: Peri<'static, PB11>,
    rtc_peripheral: Peri<'static, RTC>,
) {
    // Initialize RTC
    let (mut rtc, time_provider) = Rtc::new(rtc_peripheral, RtcConfig::default());
    
    let mut config = Config::default();
    config.memory_size = MemorySize::_32MiB;
    config.address_size = AddressSize::_24bit;
    config.prescaler = 16;
    config.cs_high_time = ChipSelectHighTime::_5Cycle;
    config.fifo_threshold = FIFOThresholdLevel::_17Bytes;
    config.sample_shifting = SampleShifting::None;
    
    let qspi_driver = Qspi::new_blocking_bank1(
        qspi,
        io0,
        io1,
        io2,
        io3,
        sck,
        cs,
        config,
    );

    // Initialize W25Q256JV flash driver
    let mut flash = W25Q256JV::new(qspi_driver);

    // Verify flash ID (W25Q256JV should return: 0xEF, 0x40, 0x19)
    let id = flash.read_id();
    info!("Flash ID: {:02X} {:02X} {:02X}", id[0], id[1], id[2]);
    
    // Verify it's a Winbond chip (Manufacturer ID 0xEF)
    if id[0] != 0xEF {
        error!("Unexpected flash manufacturer ID: 0x{:02X}", id[0]);
        return;
    }

    // Read existing datetime from flash and restore to RTC if valid
    let mut datetime_buffer = [0u8; DATETIME_DATA_SIZE];
    flash.read(TIMESTAMP_FLASH_OFFSET, &mut datetime_buffer);
    
    // Extract datetime fields from buffer
    // Layout: [year_low, year_high, month, day, hour, minute, second, reserved]
    let year = u16::from_le_bytes([datetime_buffer[0], datetime_buffer[1]]);
    let month = datetime_buffer[2];
    let day = datetime_buffer[3];
    let hour = datetime_buffer[4];
    let minute = datetime_buffer[5];
    let second = datetime_buffer[6];

    info!("Year: {:04}, Month: {:02}, Day: {:02}, Hour: {:02}, Minute: {:02}, Second: {:02}", year, month, day, hour, minute, second);
    
    // Check if data is valid (not all 0xFF or all 0x00)
    let is_valid = year != 0 && year != 0xFFFF 
        && month >= 1 && month <= 12
        && day >= 1 && day <= 31
        && hour < 24
        && minute < 60
        && second < 60;
    
    if is_valid {
        info!("Found datetime in flash: {:04}-{:02}-{:02} {:02}:{:02}:{:02}", 
            year, month, day, hour, minute, second);
        
        // Calculate day of week from date
        let day_of_week = calculate_day_of_week(year, month, day);
        
        // Create DateTime and set RTC
        match DateTime::from(year, month, day, day_of_week, hour, minute, second, 0) {
            Ok(datetime) => {
                match rtc.set_datetime(datetime) {
                    Ok(()) => {
                        info!("RTC restored from flash: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                            year, month, day, hour, minute, second);
                    }
                    Err(e) => {
                        warn!("Failed to set RTC from flash datetime: {:?}", e);
                    }
                }
            }
            Err(e) => {
                warn!("Failed to create DateTime from flash data: {:?}", e);
            }
        }
    } else {
        info!("No valid datetime found in flash, RTC will use current value");
    }

    loop {
        // Get current time from RTC
        let datetime = match time_provider.now() {
            Ok(dt) => dt,
            Err(e) => {
                warn!("Failed to read RTC: {:?}", e);
                Timer::after_secs(60).await;
                continue;
            }
        };

        // Extract datetime fields
        let year = datetime.year();
        let month = datetime.month();
        let day = datetime.day();
        let hour = datetime.hour();
        let minute = datetime.minute();
        let second = datetime.second();
        
        // Read current flash content to check if we need to update
        let mut read_buffer = [0u8; DATETIME_DATA_SIZE];
        flash.read(TIMESTAMP_FLASH_OFFSET, &mut read_buffer);
        
        // Extract current values from flash
        let current_year = u16::from_le_bytes([read_buffer[0], read_buffer[1]]);
        let current_month = read_buffer[2];
        let current_day = read_buffer[3];
        let current_hour = read_buffer[4];
        let current_minute = read_buffer[5];
        let current_second = read_buffer[6];
        
        // Check if datetime has changed
        let has_changed = year != current_year 
            || month != current_month
            || day != current_day
            || hour != current_hour
            || minute != current_minute
            || second != current_second;
        
        // Only write if datetime has changed
        if has_changed {
            // Check if sector needs erasing (if it's not all 0xFF)
            let mut sector_check = [0u8; FLASH_SECTOR_SIZE as usize];
            flash.read(TIMESTAMP_FLASH_OFFSET, &mut sector_check);
            let needs_erase = sector_check.iter().any(|&b| b != 0xFF);
            
            if needs_erase {
                info!("Erasing sector at offset 0x{:08X}", TIMESTAMP_FLASH_OFFSET);
                flash.erase_sector(TIMESTAMP_FLASH_OFFSET);
            }
            
            // Pack datetime fields into buffer
            // Layout: [year_low, year_high, month, day, hour, minute, second, reserved]
            let year_bytes = year.to_le_bytes();
            let mut datetime_bytes = [0u8; DATETIME_DATA_SIZE];
            datetime_bytes[0] = year_bytes[0];
            datetime_bytes[1] = year_bytes[1];
            datetime_bytes[2] = month;
            datetime_bytes[3] = day;
            datetime_bytes[4] = hour;
            datetime_bytes[5] = minute;
            datetime_bytes[6] = second;
            datetime_bytes[7] = 0; // reserved
            
            flash.write(TIMESTAMP_FLASH_OFFSET, &datetime_bytes);
            
            info!("Datetime written to flash: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                year, month, day, hour, minute, second);
        } else {
            info!("Datetime unchanged: {:04}-{:02}-{:02} {:02}:{:02}:{:02}",
                year, month, day, hour, minute, second);
        }

        Timer::after_secs(60).await; // Update datetime every 60 seconds
    }
}

/// Calculate day of week from date using Zeller's congruence
/// Returns DayOfWeek enum value
/// DayOfWeek enum: Monday=1, Tuesday=2, Wednesday=3, Thursday=4, Friday=5, Saturday=6, Sunday=7
fn calculate_day_of_week(year: u16, month: u8, day: u8) -> DayOfWeek {
    // Zeller's congruence algorithm
    let mut m = month as i32;
    let mut y = year as i32;
    
    // Adjust month and year for Zeller's formula (Jan=13, Feb=14 of previous year)
    if m < 3 {
        m += 12;
        y -= 1;
    }
    
    let k = y % 100; // year of century
    let j = y / 100; // century
    
    // Zeller's congruence: h = (q + floor(13(m+1)/5) + k + floor(k/4) + floor(j/4) - 2j) mod 7
    let q = day as i32;
    let h = (q + (13 * (m + 1)) / 5 + k + k / 4 + j / 4 - 2 * j) % 7;
    
    // Convert to ISO 8601 day of week (1=Monday, 7=Sunday)
    // Zeller's gives: 0=Saturday, 1=Sunday, 2=Monday, ..., 6=Friday
    let iso_dow = ((h + 5) % 7) + 1;
    
    match iso_dow {
        1 => DayOfWeek::Monday,
        2 => DayOfWeek::Tuesday,
        3 => DayOfWeek::Wednesday,
        4 => DayOfWeek::Thursday,
        5 => DayOfWeek::Friday,
        6 => DayOfWeek::Saturday,
        7 => DayOfWeek::Sunday,
        _ => DayOfWeek::Monday, // Should never happen
    }
}
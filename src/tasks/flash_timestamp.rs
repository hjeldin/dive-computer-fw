use defmt::{info, warn, error};
use embassy_stm32::peripherals::{PA6, PA7, PB0, PB1, PB10, PB11, QUADSPI};
use embassy_stm32::qspi::enums::{AddressSize, ChipSelectHighTime, DummyCycles, FIFOThresholdLevel, MemorySize, QspiWidth, SampleShifting};
use embassy_stm32::qspi::{Config, Qspi, TransferConfig};
use embassy_stm32::peripherals::RTC;
use embassy_stm32::rtc::{DateTime, DateTimeError, DayOfWeek, Rtc, RtcContainer, RtcTimeProvider};
use embassy_stm32::Peri;
use embassy_futures::select::{select, Either};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::once_lock::OnceLock;
use embassy_sync::signal::Signal;
use embassy_time::{Duration, Instant, Timer};
use crate::LOW_POWER_MODE;
use core::sync::atomic::Ordering;

// QUADSPI flash address for storing timestamp
// QUADSPI flash is memory-mapped at 0x90000000
// Winbond W25Q256JV: 256Mbit (32MB) = 0x2000000 bytes
// Above 16MB, so the chip must be in 4-byte address mode (see `W25Q256JV::new`).
const TIMESTAMP_FLASH_OFFSET: u32 = 0x01FFC000; // 16KB below the end of the 32MB flash
// User settings, in the sector after the timestamps. Same slot log layout.
const SETTINGS_FLASH_OFFSET: u32 = TIMESTAMP_FLASH_OFFSET + FLASH_SECTOR_SIZE;

// Timestamp slot: year (u16), month (u8), day (u8), hour (u8), minute (u8), second (u8), marker (u8) = 8 bytes
// Layout: [year_low, year_high, month, day, hour, minute, second, SLOT_MARKER]
// Each sector holds a log of slots, see `SlotLog`.
// W25Q256JV specifications:
// - 4KB sectors (erase unit)
// - 256-byte pages (program unit)
const FLASH_PAGE_SIZE: usize = 256;
const FLASH_SECTOR_SIZE: u32 = 4096; // 4KB sectors
const SLOT_SIZE: usize = 8; // 8 bytes per record
const SLOTS_PER_SECTOR: usize = FLASH_SECTOR_SIZE as usize / SLOT_SIZE;
// Last byte of a written slot; an erased byte reads 0xFF
const SLOT_MARKER: u8 = 0xA5;

// W25Q256JV Flash Commands
const CMD_READ: u8 = 0x03;                    // Standard Read
const CMD_FAST_READ: u8 = 0x0B;              // Fast Read
const CMD_FAST_READ_QUAD_OUTPUT: u8 = 0x6B;  // Fast Read Quad Output (1-1-4)
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
const CMD_READ_STATUS_REG3: u8 = 0x15;       // Read Status Register 3
const CMD_WRITE_STATUS_REG: u8 = 0x01;       // Write Status Register (SR1 and SR2)
const CMD_WRITE_STATUS_REG2: u8 = 0x31;      // Write Status Register 2 only
const CMD_READ_ID: u8 = 0x9F;                 // Read JEDEC ID
const CMD_ENABLE_RESET: u8 = 0x66;            // Enable Reset
const CMD_RESET: u8 = 0x99;                  // Reset Device
const CMD_ENTER_4BYTE_MODE: u8 = 0xB7;       // Enter 4-Byte Address Mode

// Status Register bits
const SR1_BUSY: u8 = 0x01;                   // Write in progress
const SR1_WEL: u8 = 0x02;                    // Write enable latch
const SR2_QE: u8 = 0x02;                     // Quad Enable (bit 1)
const SR3_ADS: u8 = 0x01;                    // Current address mode is 4-byte

// Reset time (tRST) of the W25Q256JV, max 30us
const RESET_TIME_US: u64 = 30;

// Max operation times from the W25Q256JV datasheet
const RESET_TIMEOUT: Duration = Duration::from_millis(1);
const PAGE_PROGRAM_TIMEOUT: Duration = Duration::from_millis(3);
const WRITE_STATUS_TIMEOUT: Duration = Duration::from_millis(15);
const SECTOR_ERASE_TIMEOUT: Duration = Duration::from_millis(400);
const BLOCK_ERASE_32K_TIMEOUT: Duration = Duration::from_millis(1600);
const BLOCK_ERASE_64K_TIMEOUT: Duration = Duration::from_millis(2000);
const CHIP_ERASE_TIMEOUT: Duration = Duration::from_secs(400);
// How often `wait_ready` polls the BUSY bit; other tasks run in between
const BUSY_POLL_INTERVAL: Duration = Duration::from_micros(100);

/// Reads the RTC for other tasks, once this task has started it.
static TIME_PROVIDER: OnceLock<RtcTimeProvider> = OnceLock::new();
/// A date and time for the RTC, chosen in the settings scene.
static SET_DATETIME: Signal<CriticalSectionRawMutex, DateTime> = Signal::new();

/// The RTC's date and time, or `None` before the RTC starts or if it holds no valid time.
pub fn rtc_now() -> Option<DateTime> {
    TIME_PROVIDER.try_get()?.now().ok()
}

/// Set the RTC to the given date and time, at second 0. The flash log
/// records the new time right away.
pub fn set_datetime(year: u16, month: u8, day: u8, hour: u8, minute: u8) {
    let day_of_week = calculate_day_of_week(year, month, day);
    match DateTime::from(year, month, day, day_of_week, hour, minute, 0, 0) {
        Ok(datetime) => SET_DATETIME.signal(datetime),
        Err(e) => warn!("Invalid datetime from settings: {:?}", e),
    }
}

fn set_rtc(rtc: &RtcContainer, datetime: DateTime) {
    match critical_section::with(|cs| rtc.borrow_mut(cs).set_datetime(datetime)) {
        Ok(()) => info!("RTC set from settings"),
        Err(e) => warn!("Failed to set RTC from settings: {:?}", e),
    }
}

/// Settings saved in flash, as the settings scene shows them.
#[derive(Clone, Copy, PartialEq, Eq, defmt::Format)]
pub struct StoredSettings {
    /// Gradient factors in percent.
    pub gf_low: u8,
    pub gf_high: u8,
    /// Estimated SAC rate in litres per minute.
    pub sac_l_min: u8,
    /// Index into `stm_graphics::UsbMode::ALL`.
    pub usb_mode: u8,
    /// Backlight in percent, a multiple of `stm_graphics::BACKLIGHT_STEP`.
    pub backlight_percent: u8,
    /// Idle time before the screen turns off, one of
    /// `stm_graphics::SCREEN_OFF_OPTIONS_S`; 0 keeps it on.
    pub screen_off_s: u16,
}

/// Settings to save, from the settings scene.
static SAVE_SETTINGS: Signal<CriticalSectionRawMutex, StoredSettings> = Signal::new();
/// Settings read from flash at boot, for the screen task. Not signalled when
/// flash holds none, so the defaults stay.
pub static LOADED_SETTINGS: Signal<CriticalSectionRawMutex, StoredSettings> = Signal::new();

/// Save settings to flash. Several calls before the write keep only the last.
pub fn save_settings(settings: StoredSettings) {
    SAVE_SETTINGS.signal(settings);
}

/// Wait up to `duration`. A new datetime from the settings scene is set on the
/// RTC and ends the wait early, so the timestamp log records it; settings to
/// save are written to `settings_log` and the wait goes on.
async fn wait_or_apply(
    rtc: &RtcContainer,
    mut flash_logs: Option<(&mut W25Q256JV, &mut SlotLog<StoredSettings>)>,
    duration: Duration,
) {
    let deadline = Instant::now() + duration;
    loop {
        let event = select(
            Timer::at(deadline),
            select(SET_DATETIME.wait(), SAVE_SETTINGS.wait()),
        )
        .await;
        match event {
            Either::First(()) => return,
            Either::Second(Either::First(datetime)) => {
                set_rtc(rtc, datetime);
                return;
            }
            Either::Second(Either::Second(settings)) => match flash_logs.as_mut() {
                Some((flash, log)) if log.latest != Some(settings) => match log.append(flash, settings).await {
                    Ok(()) => info!("Settings written to flash: {}", settings),
                    Err(e) => warn!("Failed to write settings to flash: {}", e),
                },
                Some(_) => {}
                None => warn!("No flash, settings not saved"),
            },
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, defmt::Format)]
pub enum FlashError {
    /// BUSY did not clear within the datasheet maximum for the operation
    Timeout,
}

/// W25Q256JV Flash Memory Driver
pub struct W25Q256JV {
    qspi: Qspi<'static, QUADSPI, embassy_stm32::mode::Blocking>,
}

impl W25Q256JV {
    /// Create a new W25Q256JV flash driver
    pub async fn new(qspi: Qspi<'static, QUADSPI, embassy_stm32::mode::Blocking>) -> Result<Self, FlashError> {
        let mut flash = Self { qspi };
        
        // Initialize flash: reset, enable quad mode and 4-byte addressing.
        // The QSPI peripheral must be configured with `AddressSize::_32bit`.
        flash.reset().await?;
        flash.enable_quad_mode().await?;
        flash.enter_4byte_mode();
        
        Ok(flash)
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
    async fn write_register(&mut self, cmd: u8, value: u8) -> Result<(), FlashError> {
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
        self.wait_ready(WRITE_STATUS_TIMEOUT).await
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

    /// Wait for flash to be ready (not busy), yielding to other tasks between polls
    pub async fn wait_ready(&mut self, timeout: Duration) -> Result<(), FlashError> {
        let deadline = Instant::now() + timeout;
        loop {
            if !self.is_busy() {
                return Ok(());
            }
            if Instant::now() >= deadline {
                return Err(FlashError::Timeout);
            }
            Timer::after(BUSY_POLL_INTERVAL).await;
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
    pub async fn enable_quad_mode(&mut self) -> Result<(), FlashError> {
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
            self.wait_ready(WRITE_STATUS_TIMEOUT).await?;
            info!("Quad mode enabled");
        } else {
            info!("Quad mode already enabled");
        }
        Ok(())
    }

    /// Enter 4-byte address mode, needed to reach addresses above 16MB
    pub fn enter_4byte_mode(&mut self) {
        self.exec_command(CMD_ENTER_4BYTE_MODE);
        if self.read_register(CMD_READ_STATUS_REG3) & SR3_ADS == 0 {
            error!("Flash did not enter 4-byte address mode");
        } else {
            info!("4-byte address mode enabled");
        }
    }

    /// Reset the flash device. This also returns it to 3-byte address mode.
    pub async fn reset(&mut self) -> Result<(), FlashError> {
        self.exec_command(CMD_ENABLE_RESET);
        self.exec_command(CMD_RESET);
        // The device ignores commands until the reset completes
        embassy_time::block_for(Duration::from_micros(RESET_TIME_US));
        self.wait_ready(RESET_TIMEOUT).await?;
        info!("Flash reset complete");
        Ok(())
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

    /// Read data from flash using Fast Read Quad Output
    pub fn read(&mut self, address: u32, buffer: &mut [u8]) {
        let transaction = TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::QUAD,
            instruction: CMD_FAST_READ_QUAD_OUTPUT,
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

    /// Erase a spare sector, program a pattern with single and quad page
    /// program, and log the erase time and what reads back. A working chip
    /// stays busy for tens of ms on the erase and returns the pattern.
    #[cfg(feature = "flash-selftest")]
    pub async fn selftest(&mut self) {
        const BASE: u32 = 0x01FFE000;
        let pattern: [u8; 8] = [0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF];

        // SFDP takes a 3-byte address and 8 dummy clocks; the 32-bit address
        // phase of 0 covers both. A genuine part starts with "SFDP".
        let mut sfdp = [0u8; 16];
        self.qspi.blocking_read(&mut sfdp, TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::SING,
            instruction: 0x5A,
            address: Some(0),
            dummy: DummyCycles::_0,
        });
        // Unique ID: five dummy bytes in 4-byte address mode, then 8 ID bytes
        let mut unique_id = [0u8; 8];
        self.qspi.blocking_read(&mut unique_id, TransferConfig {
            iwidth: QspiWidth::SING,
            awidth: QspiWidth::SING,
            dwidth: QspiWidth::SING,
            instruction: 0x4B,
            address: Some(0),
            dummy: DummyCycles::_8,
        });
        let sr = (self.read_status_reg1(), self.read_register(CMD_READ_STATUS_REG2), self.read_register(CMD_READ_STATUS_REG3));
        info!("[FlashSelftest] SFDP {=[u8]:02x}", sfdp);
        info!("[FlashSelftest] unique ID {=[u8]:02x}, SR1 0x{:02X} SR2 0x{:02X} SR3 0x{:02X}", unique_id, sr.0, sr.1, sr.2);

        let start = Instant::now();
        let erased = self.erase_sector(BASE).await;
        let erase_us = start.elapsed().as_micros();
        let mut after_erase = [0u8; 8];
        self.read_standard(BASE, &mut after_erase);
        info!(
            "[FlashSelftest] erase ok {} in {} us, reads {=[u8]:02x}",
            erased.is_ok(), erase_us, after_erase
        );

        for (name, cmd, dwidth, address) in [
            ("single", CMD_PAGE_PROGRAM, QspiWidth::SING, BASE),
            ("quad", CMD_QUAD_PAGE_PROGRAM, QspiWidth::QUAD, BASE + FLASH_PAGE_SIZE as u32),
        ] {
            let transaction = TransferConfig {
                iwidth: QspiWidth::SING,
                awidth: QspiWidth::SING,
                dwidth,
                instruction: cmd,
                address: Some(address),
                dummy: DummyCycles::_0,
            };
            self.enable_write();
            let start = Instant::now();
            self.qspi.blocking_write(&pattern, transaction);
            let ready = self.wait_ready(PAGE_PROGRAM_TIMEOUT).await;
            let program_us = start.elapsed().as_micros();
            let (mut single, mut quad) = ([0u8; 8], [0u8; 8]);
            self.read_standard(address, &mut single);
            self.read(address, &mut quad);
            let pass = single == pattern && quad == pattern;
            info!(
                "[FlashSelftest] {} program ok {} in {} us, read single {=[u8]:02x} quad {=[u8]:02x}: {}",
                name, ready.is_ok(), program_us, single, quad, if pass { "PASS" } else { "FAIL" }
            );
        }
    }

    /// Write a single page (256 bytes) to flash
    /// Note: address must be page-aligned (address % 256 == 0)
    /// and data length must not exceed page boundary
    async fn write_page(&mut self, address: u32, data: &[u8]) -> Result<(), FlashError> {
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
        self.wait_ready(PAGE_PROGRAM_TIMEOUT).await
    }

    /// Write data to flash (handles page boundaries automatically)
    pub async fn write(&mut self, address: u32, data: &[u8]) -> Result<(), FlashError> {
        let mut offset = 0;
        let mut current_addr = address;

        while offset < data.len() {
            // Calculate how much we can write in this page
            let page_offset = (current_addr as usize) % FLASH_PAGE_SIZE;
            let remaining_in_page = FLASH_PAGE_SIZE - page_offset;
            let remaining_data = data.len() - offset;
            let write_len = remaining_in_page.min(remaining_data);

            let chunk = &data[offset..offset + write_len];
            self.write_page(current_addr, chunk).await?;

            offset += write_len;
            current_addr += write_len as u32;
        }
        Ok(())
    }

    /// Erase a 4KB sector
    pub async fn erase_sector(&mut self, address: u32) -> Result<(), FlashError> {
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
        self.wait_ready(SECTOR_ERASE_TIMEOUT).await
    }

    /// Erase a 32KB block
    pub async fn erase_block_32k(&mut self, address: u32) -> Result<(), FlashError> {
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
        self.wait_ready(BLOCK_ERASE_32K_TIMEOUT).await
    }

    /// Erase a 64KB block
    pub async fn erase_block_64k(&mut self, address: u32) -> Result<(), FlashError> {
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
        self.wait_ready(BLOCK_ERASE_64K_TIMEOUT).await
    }

    /// Erase entire chip (use with caution!)
    pub async fn erase_chip(&mut self) -> Result<(), FlashError> {
        self.enable_write();
        self.exec_command(CMD_CHIP_ERASE);
        self.wait_ready(CHIP_ERASE_TIMEOUT).await?;
        warn!("Chip erase completed");
        Ok(())
    }
}

/// Date and time as stored in flash. Field order makes the derived `Ord` chronological.
#[derive(Clone, Copy, PartialEq, Eq, PartialOrd, Ord, defmt::Format)]
struct Timestamp {
    year: u16,
    month: u8,
    day: u8,
    hour: u8,
    minute: u8,
    second: u8,
}

/// A record that fits one flash slot.
trait Slot: Copy + PartialEq + defmt::Format {
    /// The last byte must be `SLOT_MARKER`, so an erased slot never decodes.
    fn encode(&self) -> [u8; SLOT_SIZE];
    /// Returns `None` for an erased, partly written or corrupt slot.
    fn decode(slot: &[u8]) -> Option<Self>;
}

impl Timestamp {
    fn from_datetime(dt: &DateTime) -> Self {
        Self {
            year: dt.year(),
            month: dt.month(),
            day: dt.day(),
            hour: dt.hour(),
            minute: dt.minute(),
            second: dt.second(),
        }
    }

    fn to_datetime(&self) -> Result<DateTime, DateTimeError> {
        let day_of_week = calculate_day_of_week(self.year, self.month, self.day);
        DateTime::from(self.year, self.month, self.day, day_of_week, self.hour, self.minute, self.second, 0)
    }
}

impl Slot for Timestamp {
    fn encode(&self) -> [u8; SLOT_SIZE] {
        let year = self.year.to_le_bytes();
        [year[0], year[1], self.month, self.day, self.hour, self.minute, self.second, SLOT_MARKER]
    }

    fn decode(slot: &[u8]) -> Option<Self> {
        let t = Self {
            year: u16::from_le_bytes([slot[0], slot[1]]),
            month: slot[2],
            day: slot[3],
            hour: slot[4],
            minute: slot[5],
            second: slot[6],
        };
        let valid = slot[7] == SLOT_MARKER
            && (2000..=2099).contains(&t.year)
            && (1..=12).contains(&t.month)
            && (1..=31).contains(&t.day)
            && t.hour < 24
            && t.minute < 60
            && t.second < 60;
        valid.then_some(t)
    }
}

/// Layout version of `StoredSettings` slots. Version 1 slots, written before
/// the screen settings existed, are still read; other versions are ignored.
const SETTINGS_VERSION: u8 = 2;

impl Slot for StoredSettings {
    // Layout: [version, gf_low, gf_high, sac_l_min, usb_mode, screen, checksum, SLOT_MARKER].
    // `screen` holds the backlight in tens of percent in the high nibble and the
    // index of the screen-off time in the low nibble; version 1 has 0xFF there.
    fn encode(&self) -> [u8; SLOT_SIZE] {
        let backlight_tens = self.backlight_percent / 10;
        let screen_off_index = stm_graphics::screen_off_index(self.screen_off_s).unwrap_or(0) as u8;
        let screen = backlight_tens << 4 | screen_off_index;
        let mut slot = [SETTINGS_VERSION, self.gf_low, self.gf_high, self.sac_l_min, self.usb_mode, screen, 0, SLOT_MARKER];
        slot[6] = checksum(&slot[..6]);
        slot
    }

    fn decode(slot: &[u8]) -> Option<Self> {
        let defaults = stm_graphics::Settings::new();
        let (backlight_percent, screen_off_s) = match slot[0] {
            1 => (defaults.backlight_percent, defaults.screen_off_s),
            SETTINGS_VERSION => {
                let options = &stm_graphics::SCREEN_OFF_OPTIONS_S;
                ((slot[5] >> 4) * 10, *options.get((slot[5] & 0x0F) as usize)?)
            }
            _ => return None,
        };
        let settings = Self {
            gf_low: slot[1],
            gf_high: slot[2],
            sac_l_min: slot[3],
            usb_mode: slot[4],
            backlight_percent,
            screen_off_s,
        };
        let valid = slot[7] == SLOT_MARKER
            && slot[6] == checksum(&slot[..6])
            && (stm_graphics::BACKLIGHT_MIN..=stm_graphics::BACKLIGHT_MAX).contains(&settings.backlight_percent)
            && (stm_graphics::GF_MIN..=stm_graphics::GF_MAX).contains(&settings.gf_low)
            && (settings.gf_low..=stm_graphics::GF_MAX).contains(&settings.gf_high)
            && (stm_graphics::SAC_MIN..=stm_graphics::SAC_MAX).contains(&settings.sac_l_min)
            && stm_graphics::UsbMode::from_index(settings.usb_mode as usize).is_some();
        valid.then_some(settings)
    }
}

/// Sum of `bytes` plus one, so all-zero data does not checksum to zero.
fn checksum(bytes: &[u8]) -> u8 {
    bytes.iter().fold(1u8, |sum, &b| sum.wrapping_add(b))
}

/// Append-only log of records in one sector. Each write takes the next free slot; the sector
/// is erased only when all slots are used, so one erase covers `SLOTS_PER_SECTOR` writes.
struct SlotLog<T: Slot> {
    offset: u32,
    latest: Option<T>,
    next_slot: usize,
}

impl<T: Slot> SlotLog<T> {
    /// Finds the newest valid slot and the first erased slot in the sector at `offset`.
    fn scan(flash: &mut W25Q256JV, offset: u32) -> Self {
        let mut latest = None;
        let mut chunk = [0u8; FLASH_PAGE_SIZE];
        let slots_per_chunk = FLASH_PAGE_SIZE / SLOT_SIZE;
        for chunk_index in 0..SLOTS_PER_SECTOR / slots_per_chunk {
            flash.read(offset + (chunk_index * FLASH_PAGE_SIZE) as u32, &mut chunk);
            for (i, slot) in chunk.chunks_exact(SLOT_SIZE).enumerate() {
                if slot.iter().all(|&b| b == 0xFF) {
                    return Self { offset, latest, next_slot: chunk_index * slots_per_chunk + i };
                }
                // A slot cut short by a power loss is skipped; the next slot is still usable.
                if let Some(t) = T::decode(slot) {
                    latest = Some(t);
                }
            }
        }
        Self { offset, latest, next_slot: SLOTS_PER_SECTOR }
    }

    async fn append(&mut self, flash: &mut W25Q256JV, t: T) -> Result<(), FlashError> {
        if self.next_slot >= SLOTS_PER_SECTOR {
            info!("Slot sector full, erasing 0x{:08X}", self.offset);
            flash.erase_sector(self.offset).await?;
            self.next_slot = 0;
        }
        let address = self.offset + (self.next_slot * SLOT_SIZE) as u32;
        // Advance first: if the write fails, the slot may be partly programmed and must not be reused.
        self.next_slot += 1;
        flash.write(address, &t.encode()).await?;
        self.latest = Some(t);
        Ok(())
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
    let (rtc, time_provider) = Rtc::new(rtc_peripheral);
    let _ = TIME_PROVIDER.init(time_provider.clone());
    
    let mut config = Config::default();
    config.memory_size = MemorySize::_32MiB;
    config.address_size = AddressSize::_32bit;
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
    let mut flash = match W25Q256JV::new(qspi_driver).await {
        Ok(flash) => flash,
        Err(e) => {
            error!("Flash init failed: {}", e);
            // The clock can still be set, it is just not saved
            loop {
                wait_or_apply(&rtc, None, Duration::from_secs(3600)).await;
            }
        }
    };

    // Verify flash ID (W25Q256JV should return: 0xEF, 0x40, 0x19)
    let id = flash.read_id();
    info!("Flash ID: {:02X} {:02X} {:02X}", id[0], id[1], id[2]);
    
    // Verify it's a Winbond chip (Manufacturer ID 0xEF)
    if id[0] != 0xEF {
        error!("Unexpected flash manufacturer ID: 0x{:02X}", id[0]);
        loop {
            wait_or_apply(&rtc, None, Duration::from_secs(3600)).await;
        }
    }

    #[cfg(feature = "flash-selftest")]
    flash.selftest().await;

    let mut log = SlotLog::<Timestamp>::scan(&mut flash, TIMESTAMP_FLASH_OFFSET);
    info!("Timestamp log: next slot {}/{}", log.next_slot, SLOTS_PER_SECTOR);
    let mut settings_log = SlotLog::<StoredSettings>::scan(&mut flash, SETTINGS_FLASH_OFFSET);
    info!("Settings log: next slot {}/{}", settings_log.next_slot, SLOTS_PER_SECTOR);
    match settings_log.latest {
        Some(settings) => {
            info!("Settings from flash: {}", settings);
            crate::dive::GRADIENT_FACTORS.signal((settings.gf_low, settings.gf_high));
            LOADED_SETTINGS.signal(settings);
        }
        None => info!("No settings in flash, using defaults"),
    }

    // Restore the RTC from flash, unless the RTC kept running and is already at or past that time.
    match log.latest {
        Some(stored) => {
            info!("Found datetime in flash: {}", stored);
            let rtc_now = time_provider.now().ok().map(|dt| Timestamp::from_datetime(&dt));
            if rtc_now.is_some_and(|now| now >= stored) {
                info!("RTC is at or past the stored datetime, keeping it");
            } else {
                match stored.to_datetime() {
                    Ok(datetime) => match critical_section::with(|cs| rtc.borrow_mut(cs).set_datetime(datetime)) {
                        Ok(()) => info!("RTC restored from flash: {}", stored),
                        Err(e) => warn!("Failed to set RTC from flash datetime: {:?}", e),
                    },
                    Err(e) => warn!("Failed to create DateTime from flash data: {:?}", e),
                }
            }
        }
        None => info!("No valid datetime found in flash, RTC will use current value"),
    }

    loop {
        // Check for low power mode - skip QUADSPI operations but keep RTC running
        if LOW_POWER_MODE.load(Ordering::Relaxed) {
            info!("[FlashTimestamp] Low power mode - skipping flash writes, RTC continues");
            // Wait for low power mode to be cleared
            loop {
                Timer::after_secs(60).await;
                if !LOW_POWER_MODE.load(Ordering::Relaxed) {
                    info!("[FlashTimestamp] Waking from low power mode");
                    break;
                }
            }
            continue;
        }

        match time_provider.now() {
            Ok(datetime) => {
                let now = Timestamp::from_datetime(&datetime);
                if log.latest == Some(now) {
                    info!("Datetime unchanged: {}", now);
                } else {
                    match log.append(&mut flash, now).await {
                        Ok(()) => info!("Datetime written to flash: {}", now),
                        Err(e) => warn!("Failed to write datetime to flash: {}", e),
                    }
                }
            }
            Err(e) => warn!("Failed to read RTC: {:?}", e),
        }

        // Update datetime every 60 seconds, or right after the settings change it
        wait_or_apply(&rtc, Some((&mut flash, &mut settings_log)), Duration::from_secs(60)).await;
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
    // rem_euclid: the sum can be negative, and `%` would then return a negative remainder
    let h = (q + (13 * (m + 1)) / 5 + k + k / 4 + j / 4 - 2 * j).rem_euclid(7);
    
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
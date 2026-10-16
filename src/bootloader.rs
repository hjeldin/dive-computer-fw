//! App side of the USB bootloader: request update mode, confirm a swapped-in image.
//! Design: docs/superpowers/specs/2026-10-16-usb-bootloader-design.md

use core::cell::RefCell;

use dc_update_protocol::{UPDATE_REQUEST_BKP, UPDATE_REQUESTED};
use defmt::{info, warn};
use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterConfig};
use embassy_stm32::Peri;
use embassy_stm32::flash::{Flash, WRITE_SIZE};
use embassy_stm32::pac;
use embassy_stm32::peripherals::FLASH;
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;

/// Asks the bootloader to stay in update mode, then resets.
pub fn reboot_to_bootloader() -> ! {
    info!("Rebooting into bootloader update mode");
    pac::RCC.apb1enr1().modify(|w| w.set_pwren(true));
    pac::PWR.cr1().modify(|w| w.set_dbp(true));
    pac::RTC.bkpr(UPDATE_REQUEST_BKP).write(|w| w.set_bkp(UPDATE_REQUESTED));
    cortex_m::peripheral::SCB::sys_reset()
}

/// Confirms the running image, so the bootloader does not roll it back.
pub fn mark_booted(flash: Peri<'static, FLASH>) {
    let flash = Mutex::<NoopRawMutex, _>::new(RefCell::new(Flash::new_blocking(flash)));
    let config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
    let mut magic = AlignedBuffer([0u8; WRITE_SIZE]);
    let mut state = BlockingFirmwareState::from_config(config, &mut magic.0);
    match state.mark_booted() {
        Ok(()) => info!("Firmware confirmed"),
        Err(e) => warn!("mark_booted failed: {}", defmt::Debug2Format(&e)),
    }
}

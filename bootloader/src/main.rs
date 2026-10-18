//! Dive computer bootloader: embassy-boot A/B swap plus USB bulk update mode.
//! Design: docs/superpowers/specs/2026-10-16-usb-bootloader-design.md
#![no_std]
#![no_main]

mod boot_request;
mod flash_target;
mod usb_update;

use core::cell::RefCell;

use cortex_m_rt::entry;
use defmt::info;
use embassy_boot_stm32::{BootLoader, BootLoaderConfig};
use embassy_stm32::flash::Flash;
use embassy_sync::blocking_mutex::Mutex;

use crate::flash_target::SharedFlash;
use {defmt_rtt as _, panic_probe as _};

const FLASH_BASE: u32 = 0x0800_0000;

#[entry]
fn main() -> ! {
    let update = boot_request::take_update_request() || boot_request::enter_held();
    let mut clocks = if update { usb_update::clock_config() } else { Default::default() };
    // Must match the app's RTC clock: init resets the backup domain whenever the
    // configured source differs from the running one, which clears the RTC
    clocks.rcc.ls = embassy_stm32::rcc::LsConfig::default_lse();
    let p = embassy_stm32::init(clocks);
    let flash: SharedFlash = Mutex::new(RefCell::new(Flash::new_blocking(p.FLASH)));

    // Always finish a pending swap, or revert an unconfirmed image, before anything else. An upload
    // must never start on top of an unconfirmed swap: embassy-boot would later "revert" it and mix
    // the new image into ACTIVE, losing both images.
    let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
    let active = FLASH_BASE + config.active.offset();
    let bl = BootLoader::prepare::<_, _, _, 2048>(config);
    info!("Boot state {}", bl.state);

    if update {
        usb_update::run(&flash, p.USB_OTG_FS, p.PA12, p.PA11);
    }
    // Checked after `prepare` so a first install (empty ACTIVE, image in DFU) boots after its swap.
    if !boot_request::image_valid(active) {
        info!("No valid image at 0x{:08x}, entering update mode", active);
        boot_request::request_update_and_reset();
    }
    unsafe { bl.load(active) }
}

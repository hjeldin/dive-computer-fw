//! Decides whether to stay in update mode. Runs on reset-state clocks (MSI 4 MHz),
//! before `embassy_stm32::init`, using the PAC directly.

use dc_update_protocol::{UPDATE_REQUEST_BKP, UPDATE_REQUESTED};
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::{Idr, Moder, Pupdr};

/// MSI runs at 4 MHz after reset.
const CYCLES_PER_MS: u32 = 4_000;
const ENTER_PIN: usize = 4; // PA4

fn write_bkp(value: u32) {
    pac::RCC.apb1enr1().modify(|w| w.set_pwren(true));
    pac::PWR.cr1().modify(|w| w.set_dbp(true));
    pac::RTC.bkpr(UPDATE_REQUEST_BKP).write(|w| w.set_bkp(value));
}

/// True (once) if the app asked for update mode before resetting.
pub fn take_update_request() -> bool {
    let requested = pac::RTC.bkpr(UPDATE_REQUEST_BKP).read().bkp() == UPDATE_REQUESTED;
    if requested {
        write_bkp(0);
    }
    requested
}

/// Requests update mode and resets. Used when no bootable image is found after `prepare`.
pub fn request_update_and_reset() -> ! {
    write_bkp(UPDATE_REQUESTED);
    cortex_m::peripheral::SCB::sys_reset()
}

/// True if ENTER (PA4, active low) reads pressed on 5 samples 2 ms apart.
pub fn enter_held() -> bool {
    pac::RCC.ahb2enr().modify(|w| w.set_gpioaen(true));
    pac::GPIOA.pupdr().modify(|w| w.set_pupdr(ENTER_PIN, Pupdr::PULL_UP));
    pac::GPIOA.moder().modify(|w| w.set_moder(ENTER_PIN, Moder::INPUT));
    (0..5).all(|_| {
        cortex_m::asm::delay(2 * CYCLES_PER_MS);
        pac::GPIOA.idr().read().idr(ENTER_PIN) == Idr::LOW
    })
}

/// True if the image at `address` starts with a stack pointer inside SRAM1.
pub fn image_valid(address: u32) -> bool {
    let initial_sp = unsafe { core::ptr::read_volatile(address as *const u32) };
    (0x2000_0000..=0x2001_8000).contains(&initial_sp)
}

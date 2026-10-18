//! defmt global logger that sends log frames over a USB serial port, enabled
//! by the `usb-log` feature in place of `defmt-rtt`.
//!
//! Frames go into a ring buffer that the USB task drains to the log port
//! (see `tasks::usb_device`). Logging never blocks: when nobody reads the
//! port the oldest bytes are overwritten. A frame cut that way decodes as
//! garbage up to the next frame delimiter, after which the decoder resyncs.
//!
//! Decode on the host with the ELF of the running firmware:
//! `tools/usb_log.py`, or `defmt-print -e <elf> stdin < /dev/ttyACM1`.

use core::cell::{RefCell, UnsafeCell};
use core::sync::atomic::{AtomicBool, Ordering};

use critical_section::{CriticalSection, Mutex};

/// Room for the boot logs and a few seconds of `info` logs.
const BUFFER_SIZE: usize = 4096;

struct Ring {
    buffer: [u8; BUFFER_SIZE],
    /// Index of the oldest byte.
    start: usize,
    len: usize,
}

impl Ring {
    const fn new() -> Self {
        Self { buffer: [0; BUFFER_SIZE], start: 0, len: 0 }
    }

    /// Appends `bytes`, overwriting the oldest bytes when full.
    fn push(&mut self, bytes: &[u8]) {
        for &byte in bytes {
            if self.len == BUFFER_SIZE {
                self.start = (self.start + 1) % BUFFER_SIZE;
                self.len -= 1;
            }
            self.buffer[(self.start + self.len) % BUFFER_SIZE] = byte;
            self.len += 1;
        }
    }

    /// Moves the oldest bytes into `out`; returns how many.
    fn pop(&mut self, out: &mut [u8]) -> usize {
        let count = out.len().min(self.len);
        for slot in &mut out[..count] {
            *slot = self.buffer[self.start];
            self.start = (self.start + 1) % BUFFER_SIZE;
        }
        self.len -= count;
        count
    }
}

static RING: Mutex<RefCell<Ring>> = Mutex::new(RefCell::new(Ring::new()));

/// Moves pending log bytes into `out`; returns how many. Called by the USB task.
pub fn read(out: &mut [u8]) -> usize {
    critical_section::with(|cs| RING.borrow_ref_mut(cs).pop(out))
}

/// The logger holds the critical section from `acquire` to `release`, so
/// logging from an interrupt cannot interleave with a frame in progress.
struct Encoder {
    taken: AtomicBool,
    cs_restore: UnsafeCell<critical_section::RestoreState>,
    encoder: UnsafeCell<defmt::Encoder>,
}

// SAFETY: the cells are only touched between `acquire` and `release`, inside
// the critical section, and `taken` rejects reentrant use
unsafe impl Sync for Encoder {}

static ENCODER: Encoder = Encoder {
    taken: AtomicBool::new(false),
    cs_restore: UnsafeCell::new(critical_section::RestoreState::invalid()),
    encoder: UnsafeCell::new(defmt::Encoder::new()),
};

/// Pushes encoded bytes; only called while the logger holds the critical section.
fn push(bytes: &[u8]) {
    // SAFETY: the logger acquired the critical section in `acquire`
    let cs = unsafe { CriticalSection::new() };
    RING.borrow_ref_mut(cs).push(bytes);
}

#[defmt::global_logger]
struct UsbLogger;

unsafe impl defmt::Logger for UsbLogger {
    fn acquire() {
        let restore = unsafe { critical_section::acquire() };
        if ENCODER.taken.load(Ordering::Relaxed) {
            panic!("defmt logger taken reentrantly")
        }
        ENCODER.taken.store(true, Ordering::Relaxed);
        unsafe {
            ENCODER.cs_restore.get().write(restore);
            (*ENCODER.encoder.get()).start_frame(push);
        }
    }

    unsafe fn flush() {
        // The USB task sends the bytes; nothing to wait for here
    }

    unsafe fn release() {
        unsafe {
            (*ENCODER.encoder.get()).end_frame(push);
            ENCODER.taken.store(false, Ordering::Relaxed);
            critical_section::release(ENCODER.cs_restore.get().read());
        }
    }

    unsafe fn write(bytes: &[u8]) {
        unsafe { (*ENCODER.encoder.get()).write(bytes, push) }
    }
}

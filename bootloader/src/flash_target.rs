//! `UpdateTarget` writing into the embassy-boot DFU partition.

use core::cell::RefCell;

use dc_update_protocol::UpdateTarget;
use embassy_boot_stm32::{BlockingFirmwareState, FirmwareUpdaterConfig};
use embassy_embedded_hal::flash::partition::BlockingPartition;
use embassy_stm32::flash::{Blocking, Flash};
use embassy_sync::blocking_mutex::Mutex;
use embassy_sync::blocking_mutex::raw::NoopRawMutex;
use embedded_storage::nor_flash::{NorFlash, ReadNorFlash};

pub type SharedFlash = Mutex<NoopRawMutex, RefCell<Flash<'static, Blocking>>>;
type Partition<'a> = BlockingPartition<'a, NoopRawMutex, Flash<'static, Blocking>>;

pub struct FlashTarget<'a> {
    dfu: Partition<'a>,
    state: BlockingFirmwareState<'a, Partition<'a>>,
}

impl<'a> FlashTarget<'a> {
    /// `magic` must be `WRITE_SIZE` bytes and suitably aligned (use `AlignedBuffer`).
    pub fn new(flash: &'a SharedFlash, magic: &'a mut [u8]) -> Self {
        let dfu = FirmwareUpdaterConfig::from_linkerfile_blocking(flash, flash).dfu;
        let state = BlockingFirmwareState::from_config(FirmwareUpdaterConfig::from_linkerfile_blocking(flash, flash), magic);
        Self { dfu, state }
    }
}

impl UpdateTarget for FlashTarget<'_> {
    fn dfu_size(&self) -> u32 {
        self.dfu.capacity() as u32
    }

    fn page_size(&self) -> u32 {
        <Flash<'static, Blocking> as NorFlash>::ERASE_SIZE as u32
    }

    fn erase(&mut self, from: u32, to: u32) -> Result<(), ()> {
        NorFlash::erase(&mut self.dfu, from, to).map_err(drop)
    }

    fn write(&mut self, offset: u32, data: &[u8]) -> Result<(), ()> {
        NorFlash::write(&mut self.dfu, offset, data).map_err(drop)
    }

    fn read(&mut self, offset: u32, buf: &mut [u8]) -> Result<(), ()> {
        ReadNorFlash::read(&mut self.dfu, offset, buf).map_err(drop)
    }

    fn mark_updated(&mut self) -> Result<(), ()> {
        self.state.mark_updated().map_err(drop)
    }
}

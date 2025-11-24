use core::cell::RefCell;
use core::sync::atomic::{AtomicBool, Ordering};
use defmt::{info, warn};
use embassy_futures::join::join;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{FLASH, PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::{Driver, Instance};
use embassy_time::Timer;
use embassy_usb::class::hid::{HidReaderWriter, ReportId, RequestHandler};
use embassy_usb::control::OutResponse;
use embassy_usb::{Builder, Handler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;
use usbd_hid::descriptor::{KeyboardReport, SerializedDescriptor};


use embassy_boot_stm32::{AlignedBuffer, BlockingFirmwareState, FirmwareUpdaterConfig};
use embassy_stm32::flash::{Flash, WRITE_SIZE};
use embassy_sync::blocking_mutex::Mutex;
use embassy_usb_dfu::consts::DfuAttributes;
use embassy_usb_dfu::Control;
use crate::Irqs;

#[embassy_executor::task]
pub async fn usb_device_task(usb_otg_fs: Peri<'static, USB_OTG_FS>, pa12: Peri<'static, PA12>, pa11: Peri<'static, PA11>, _flash: Peri<'static, FLASH>) {
    let mut ep_out_buffer = [0u8; 256];

    // let flash = Flash::new_blocking(flash);
    // let flash = Mutex::new(RefCell::new(flash));

    // let fw_config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
    // let mut magic = AlignedBuffer([0; WRITE_SIZE]);
    // let mut firmware_state = BlockingFirmwareState::from_config(fw_config, &mut magic.0);
    // firmware_state.mark_booted().expect("Failed to mark booted");

    let mut config = embassy_stm32::usb::Config::default();

    // Do not enable vbus_detection. This is a safe default that works in all boards.
    // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
    // to enable vbus_detection to comply with the USB spec. If you enable it, the board
    // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
    config.vbus_detection = false;

    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);

    // Create embassy-usb Config
    let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
    config.max_packet_size_0 = 64;
    config.manufacturer = Some("Embassy");
    config.product = Some("USB-serial example");
    config.serial_number = Some("12345678");

    // Create embassy-usb DeviceBuilder using the driver and config.
    // It needs some buffers for building the descriptors.
    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    // let mut dfu_state = Control::new(fw_config, DfuAttributes::CAN_DOWNLOAD);

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    // Create classes on the builder.
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

    // Build the builder.
    let mut usb = builder.build();

    // Run the USB device.
    let usb_fut = usb.run();

    // Do stuff with the class!
    let echo_fut = async {
        loop {
            class.wait_connection().await;
            info!("Connected");
            let _ = echo(&mut class).await;
            info!("Disconnected");
        }
    };

    // Run everything concurrently.
    // If we had made everything `'static` above instead, we could do this using separate tasks instead.
    join(usb_fut, echo_fut).await;
}

struct Disconnected {}

impl From<EndpointError> for Disconnected {
    fn from(val: EndpointError) -> Self {
        match val {
            EndpointError::BufferOverflow => panic!("Buffer overflow"),
            EndpointError::Disabled => Disconnected {},
        }
    }
}

async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut buf = [0; 64];
    loop {
        let n = class.read_packet(&mut buf).await?;
        let data = &buf[..n];
        info!("data: {:x}", data);
        class.write_packet(data).await?;
    }
}

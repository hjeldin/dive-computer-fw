use defmt::{info, warn};
use embassy_futures::join::join;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::{Driver, Instance};
use embassy_time::Timer;
use embassy_usb::Builder;
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;

use dc_update_protocol::{APP_PID, USB_VID};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::Irqs;
use crate::bootloader::reboot_to_bootloader;

/// Line the host sends to reboot into the bootloader's update mode.
const CMD_DFU: &[u8] = b"DFU";

/// Signalled once the host has configured the device.
pub static USB_CONFIGURED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

#[embassy_executor::task]
pub async fn usb_device_task(usb_otg_fs: Peri<'static, USB_OTG_FS>, pa12: Peri<'static, PA12>, pa11: Peri<'static, PA11>) {
    let mut ep_out_buffer = [0u8; 256];

    let mut config = embassy_stm32::usb::Config::default();
    // VBUS is not routed to PA9 on this board (PA9 drives LCD_DC).
    config.vbus_detection = false;

    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);

    let mut config = embassy_usb::Config::new(USB_VID, APP_PID);
    config.max_packet_size_0 = 64;
    config.manufacturer = Some("hjeldin");
    config.product = Some("Dive Computer");
    config.serial_number = Some("00000001");

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    let mut usb = builder.build();

    let command_fut = async {
        loop {
            class.wait_connection().await;
            info!("USB CDC connected");
            USB_CONFIGURED.signal(());
            let _ = handle_commands(&mut class).await;
            info!("USB CDC disconnected");
        }
    };

    join(usb.run(), command_fut).await;
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

/// Read newline-terminated commands and act on them.
async fn handle_commands<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut packet = [0u8; 64];
    let mut line = heapless::Vec::<u8, 32>::new();
    loop {
        let n = class.read_packet(&mut packet).await?;
        for &byte in &packet[..n] {
            if byte != b'\r' && byte != b'\n' {
                // Overlong lines are not commands; drop them.
                if line.push(byte).is_err() {
                    line.clear();
                }
                continue;
            }
            if line.is_empty() {
                continue;
            }
            if line.trim_ascii() == CMD_DFU {
                class.write_packet(b"OK DFU\n").await?;
                // Let the host read the reply before USB disappears.
                Timer::after_millis(50).await;
                reboot_to_bootloader();
            }
            warn!("Unknown USB command: {=[u8]:a}", line.as_slice());
            class.write_packet(b"ERR unknown command\n").await?;
            line.clear();
        }
    }
}

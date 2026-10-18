use defmt::{info, warn};
use embassy_futures::join::join3;
#[cfg(feature = "usb-log")]
use embassy_futures::join::join;
#[cfg(feature = "usb-log")]
use embassy_usb::class::cdc_acm::Sender;
use embassy_stm32::Peri;
use embassy_stm32::peripherals::{PA11, PA12, USB_OTG_FS};
use embassy_stm32::usb::{Driver, Instance};
use embassy_futures::select::select;
use embassy_stm32::pac;
use embassy_stm32::pac::gpio::vals::Idr;
use embassy_time::{Duration, Timer};
use core::sync::atomic::{AtomicBool, Ordering};
use embassy_usb::{Builder, Handler};
use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
use embassy_usb::driver::EndpointError;

use dc_update_protocol::{APP_PID, USB_VID};
use embassy_sync::blocking_mutex::raw::CriticalSectionRawMutex;
use embassy_sync::signal::Signal;

use crate::Irqs;
use crate::bootloader::reboot_to_bootloader;

/// How often the log port checks for new log bytes when it has sent them all.
#[cfg(feature = "usb-log")]
const LOG_POLL_MS: u64 = 20;

/// Line the host sends to reboot into the bootloader's update mode.
const CMD_DFU: &[u8] = b"DFU";

/// Signalled once the host has configured the device.
pub static USB_CONFIGURED: Signal<CriticalSectionRawMutex, ()> = Signal::new();

static CONFIGURED: AtomicBool = AtomicBool::new(false);
static SUSPENDED: AtomicBool = AtomicBool::new(false);

/// True while a host has the device configured and the bus is active.
/// VBUS is not sensed, so an unplugged cable shows up as a suspended bus.
pub fn usb_connected() -> bool {
    CONFIGURED.load(Ordering::Relaxed) && !SUSPENDED.load(Ordering::Relaxed)
}

/// Tracks the bus state for `usb_connected`.
struct BusState;

impl Handler for BusState {
    fn enabled(&mut self, enabled: bool) {
        if !enabled {
            CONFIGURED.store(false, Ordering::Relaxed);
        }
    }

    fn reset(&mut self) {
        CONFIGURED.store(false, Ordering::Relaxed);
        SUSPENDED.store(false, Ordering::Relaxed);
    }

    fn configured(&mut self, configured: bool) {
        info!("USB {}", if configured { "configured" } else { "unconfigured" });
        CONFIGURED.store(configured, Ordering::Relaxed);
        if configured {
            // Turns the screen back on
            crate::tasks::lcd_brightness::note_activity();
        }
    }

    fn suspended(&mut self, suspended: bool) {
        info!("USB {}", if suspended { "suspended" } else { "resumed" });
        SUSPENDED.store(suspended, Ordering::Relaxed);
    }
}

#[embassy_executor::task]
pub async fn usb_device_task(
    mut usb_otg_fs: Peri<'static, USB_OTG_FS>,
    mut pa12: Peri<'static, PA12>,
    mut pa11: Peri<'static, PA11>,
) {
    loop {
        if !usb_power_present() {
            info!("USB power absent, USB off");
            while !usb_power_present() {
                Timer::after(POWER_POLL).await;
            }
        }
        info!("USB power present, starting USB");
        // Returns once the power goes; dropping the driver turns the USB clock
        // off, which would otherwise keep the chip out of STOP mode
        run_usb(usb_otg_fs.reborrow(), pa12.reborrow(), pa11.reborrow()).await;
        CONFIGURED.store(false, Ordering::Relaxed);
        SUSPENDED.store(false, Ordering::Relaxed);
    }
}

/// How often USB power is checked. PC0 shares EXTI line 0 with the IMU
/// interrupt on PA0, so it cannot wake the chip and is polled instead.
const POWER_POLL: Duration = Duration::from_secs(1);

/// True while USB power is present, from the charger's /PG output (MCP73871,
/// open drain, low with input power) on PC0. Read from the port directly: the
/// battery task configures the pin, and until it does the pin is analog and
/// reads low, which keeps USB up so the DFU command stays reachable.
fn usb_power_present() -> bool {
    pac::GPIOC.idr().read().idr(0) == Idr::LOW
}

async fn wait_usb_power_lost() {
    while usb_power_present() {
        Timer::after(POWER_POLL).await;
    }
    info!("USB power lost, stopping USB");
}

/// Runs the USB device until USB power goes away.
async fn run_usb(usb_otg_fs: Peri<'_, USB_OTG_FS>, pa12: Peri<'_, PA12>, pa11: Peri<'_, PA11>) {
    let mut ep_out_buffer = [0u8; 256];

    let mut config = embassy_stm32::usb::Config::default();
    // VBUS is not routed to PA9 on this board (PA9 drives LCD_DC).
    config.vbus_detection = false;

    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);

    let mut config = embassy_usb::Config::new(USB_VID, APP_PID);
    config.max_packet_size_0 = 64;
    // Two serial ports make a composite device; hosts need the
    // interface association descriptors to pair each port's interfaces
    #[cfg(feature = "usb-log")]
    {
        config.device_class = 0xEF;
        config.device_sub_class = 0x02;
        config.device_protocol = 0x01;
        config.composite_with_iads = true;
    }
    config.manufacturer = Some("hjeldin");
    config.product = Some("Dive Computer");
    config.serial_number = Some("00000001");

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];

    let mut state = State::new();
    #[cfg(feature = "usb-log")]
    let mut log_state = State::new();
    let mut bus_state = BusState;

    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [], // no msos descriptors
        &mut control_buf,
    );

    builder.handler(&mut bus_state);
    // Commands go on the first port, so it keeps interface 0
    let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);
    #[cfg(feature = "usb-log")]
    let (mut log_tx, mut log_rx) = CdcAcmClass::new(&mut builder, &mut log_state, 64).split();
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

    #[cfg(feature = "usb-log")]
    let log_fut = join(
        async {
            loop {
                log_tx.wait_connection().await;
                let _ = send_logs(&mut log_tx).await;
            }
        },
        // A tty still in cooked mode echoes the log bytes back. Unread, they
        // fill the OUT endpoint and the host blocks waiting for them to drain.
        async {
            let mut packet = [0u8; 64];
            loop {
                log_rx.wait_connection().await;
                while log_rx.read_packet(&mut packet).await.is_ok() {}
            }
        },
    );
    #[cfg(not(feature = "usb-log"))]
    let log_fut = core::future::pending::<()>();

    select(join3(usb.run(), command_fut, log_fut), wait_usb_power_lost()).await;
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

/// Send buffered defmt frames to the log port until the host closes it.
#[cfg(feature = "usb-log")]
async fn send_logs<'d, T: Instance + 'd>(class: &mut Sender<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
    let mut packet = [0u8; 64];
    loop {
        let n = crate::usb_log::read(&mut packet);
        if n == 0 {
            Timer::after_millis(LOG_POLL_MS).await;
            continue;
        }
        class.write_packet(&packet[..n]).await?;
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

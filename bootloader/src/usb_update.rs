//! Update mode: vendor-class bulk interface serving the update protocol.

use core::sync::atomic::{AtomicU32, Ordering};

use dc_update_protocol::{After, BOOTLOADER_PID, Server, USB_VID, UpdateTarget};
use defmt::{info, warn};
use embassy_boot_stm32::AlignedBuffer;
use embassy_futures::block_on;
use embassy_futures::join::join;
use embassy_stm32::flash::WRITE_SIZE;
use embassy_stm32::time::Hertz;
use embassy_stm32::usb::{self, Driver};
use embassy_stm32::{Peri, bind_interrupts, peripherals, rcc};
use embassy_usb::{Builder, Handler};
use embassy_usb::driver::{EndpointError, EndpointIn, EndpointOut};

use crate::flash_target::{FlashTarget, SharedFlash};

bind_interrupts!(struct Irqs {
    OTG_FS => usb::InterruptHandler<peripherals::USB_OTG_FS>;
});

/// Same clocks as the app: HSE 8 MHz, SYSCLK 80 MHz, USB 48 MHz from PLLSAI1 Q.
pub fn clock_config() -> embassy_stm32::Config {
    let mut config = embassy_stm32::Config::default();
    config.rcc.hse = Some(rcc::Hse { freq: Hertz(8_000_000), mode: rcc::HseMode::Oscillator });
    config.rcc.pll = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL20,
        divp: None,
        divq: None,
        divr: Some(rcc::PllRDiv::DIV2),
    });
    config.rcc.pllsai1 = Some(rcc::Pll {
        source: rcc::PllSource::HSE,
        prediv: rcc::PllPreDiv::DIV1,
        mul: rcc::PllMul::MUL12,
        divp: None,
        divq: Some(rcc::PllQDiv::DIV2),
        divr: None,
    });
    config.rcc.mux.clk48sel = rcc::mux::Clk48sel::PLLSAI1_Q;
    config.rcc.sys = rcc::Sysclk::PLL1_R;
    config
}

/// Bumped on every USB bus reset and (de)configuration, so `Server` drops state from an earlier
/// connection (e.g. a request cut off by unplugging the cable).
static SESSION: AtomicU32 = AtomicU32::new(0);

struct SessionHandler;

impl Handler for SessionHandler {
    fn reset(&mut self) {
        SESSION.fetch_add(1, Ordering::Relaxed);
    }

    fn configured(&mut self, _configured: bool) {
        SESSION.fetch_add(1, Ordering::Relaxed);
    }
}

pub fn run(
    flash: &SharedFlash,
    usb_otg_fs: Peri<'static, peripherals::USB_OTG_FS>,
    pa12: Peri<'static, peripherals::PA12>,
    pa11: Peri<'static, peripherals::PA11>,
) -> ! {
    info!("Bootloader update mode");
    let mut magic = AlignedBuffer([0u8; WRITE_SIZE]);
    let mut target = FlashTarget::new(flash, &mut magic.0);

    let mut ep_out_buffer = [0u8; 256];
    let mut usb_config = usb::Config::default();
    // VBUS is not routed to PA9 on this board.
    usb_config.vbus_detection = false;
    let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, usb_config);

    let mut config = embassy_usb::Config::new(USB_VID, BOOTLOADER_PID);
    config.manufacturer = Some("hjeldin");
    config.product = Some("Dive Computer Bootloader");
    config.serial_number = Some("00000001");
    config.max_packet_size_0 = 64;

    let mut config_descriptor = [0; 256];
    let mut bos_descriptor = [0; 256];
    let mut control_buf = [0; 64];
    let mut handler = SessionHandler;
    let mut builder = Builder::new(
        driver,
        config,
        &mut config_descriptor,
        &mut bos_descriptor,
        &mut [],
        &mut control_buf,
    );
    builder.handler(&mut handler);
    let (mut ep_out, mut ep_in) = {
        let mut function = builder.function(0xFF, 0, 0);
        let mut interface = function.interface();
        let mut alt = interface.alt_setting(0xFF, 0, 0, None);
        (alt.endpoint_bulk_out(None, 64), alt.endpoint_bulk_in(None, 64))
    };
    let mut usb = builder.build();

    block_on(join(usb.run(), serve(&mut ep_out, &mut ep_in, &mut target)));
    cortex_m::peripheral::SCB::sys_reset()
}

async fn serve(ep_out: &mut impl EndpointOut, ep_in: &mut impl EndpointIn, target: &mut impl UpdateTarget) {
    let mut server = Server::new();
    let mut packet = [0u8; 64];
    loop {
        ep_out.wait_enabled().await;
        info!("Host connected");
        loop {
            let n = match ep_out.read(&mut packet).await {
                Ok(n) => n,
                Err(EndpointError::Disabled) => break,
                Err(EndpointError::BufferOverflow) => continue,
            };
            let Some((reply, after)) = server.on_packet(target, SESSION.load(Ordering::Relaxed), &packet[..n]) else {
                continue;
            };
            if ep_in.write(reply.as_bytes()).await.is_err() {
                warn!("Reply not sent, host gone");
                break;
            }
            if after == After::Reset {
                info!("Resetting");
                // Let the host read the reply before USB disappears (~50 ms at 80 MHz).
                cortex_m::asm::delay(4_000_000);
                cortex_m::peripheral::SCB::sys_reset();
            }
        }
    }
}

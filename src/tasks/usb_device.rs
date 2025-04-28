// use defmt::info;
// use embassy_executor::{SendSpawner, Spawner};
// use embassy_futures::join::join;
// use embassy_stm32::flash::{Flash, WRITE_SIZE};
// use embassy_stm32::peripherals::{FLASH, PA11, PA12, USB_OTG_FS};
// use embassy_stm32::usb::{self, Driver, Instance};
// use embassy_time::Timer;
// use embassy_usb::{Builder, msos, UsbDevice};
// use embassy_usb::class::cdc_acm::{CdcAcmClass, State};
// use embassy_usb::driver::EndpointError;
// use embassy_usb_dfu::consts::DfuAttributes;
// use embassy_usb_dfu::{usb_dfu, Control, ResetImmediate};
// use static_cell::StaticCell;
// use embassy_boot_stm32::*;
// use core::cell::RefCell;
// use embassy_sync::blocking_mutex::Mutex;
// use crate::Irqs;

// // type MyUsbDriver = Driver<'static, USB_OTG_FS>;
// // type MyUsbDevice = UsbDevice<'static, MyUsbDriver>;

// // #[embassy_executor::task]
// // pub async fn usb_runner_task(mut usb: MyUsbDevice) -> ! {
// //     info!("usb_runner_task");
// //     loop {
// //         usb.run().await;
// //     }
// // }
// // static mut EP_OUT_BUFFER: [u8; 256] = [0u8; 256];
// // static mut config_descriptor: [u8; 256] = [0; 256];
// // static mut bos_descriptor: [u8; 256] = [0; 256];
// // static mut control_buf: [u8; 64] = [0; 64];

// const DEVICE_INTERFACE_GUIDS: &[&str] = &["{EAA9A5DC-30BA-44BC-9232-606CDC875321}"];
// #[embassy_executor::task]
// pub async unsafe fn usb_device_task(flash: FLASH, usb_otg_fs: USB_OTG_FS, pa12: PA12, pa11: PA11, spawner: Spawner) {
//     let layout = Flash::new_blocking(flash).into_blocking_regions();
//     let flash = Mutex::new(RefCell::new(layout.bank1_region));
//     #[cfg(feature = "defmt")]
//     for _ in 0..10000000 {
//         cortex_m::asm::nop();
//     }

//     let config = BootLoaderConfig::from_linkerfile_blocking(&flash, &flash, &flash);
//     let active_offset = config.active.offset();
//     let bl = BootLoader::prepare::<_, _, _, 2048>(config);

//     let mut ep_out_buffer = [0u8; 256];
//     let mut config = embassy_stm32::usb::Config::default();

//     if(bl.state == embassy_boot_stm32::State::DfuDetach) {
//         let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);
//         let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
//         config.manufacturer = Some("Embassy");
//         config.product = Some("USB-DFU Bootloader example");
//         config.serial_number = Some("1235678");

//         let fw_config = FirmwareUpdaterConfig::from_linkerfile_blocking(&flash, &flash);
//         let mut buffer = AlignedBuffer([0; WRITE_SIZE]);
//         let updater = BlockingFirmwareUpdater::new(fw_config, &mut buffer.0[..]);

//         let mut config_descriptor = [0; 256];
//         let mut bos_descriptor = [0; 256];
//         let mut control_buf = [0; 4096];
//         let mut state = Control::new(updater, DfuAttributes::CAN_DOWNLOAD);
//         let mut builder = Builder::new(
//             driver,
//             config,
//             &mut config_descriptor,
//             &mut bos_descriptor,
//             &mut [],
//             &mut control_buf,
//         );

//         // We add MSOS headers so that the device automatically gets assigned the WinUSB driver on Windows.
//         // Otherwise users need to do this manually using a tool like Zadig.
//         //
//         // It seems it is important for the DFU class that these headers be on the Device level.
//         //
//         builder.msos_descriptor(msos::windows_version::WIN8_1, 2);
//         builder.msos_feature(msos::CompatibleIdFeatureDescriptor::new("WINUSB", ""));
//         builder.msos_feature(msos::RegistryPropertyFeatureDescriptor::new(
//             "DeviceInterfaceGUIDs",
//             msos::PropertyData::RegMultiSz(DEVICE_INTERFACE_GUIDS),
//         ));

//         usb_dfu::<_, _, _, ResetImmediate, 4096>(&mut builder, &mut state);

//         let mut dev = builder.build();
//         embassy_futures::block_on(dev.run());
//     } else {
//         // Do not enable vbus_detection. This is a safe default that works in all boards.
//         // However, if your USB device is self-powered (can stay powered on if USB is unplugged), you need
//         // to enable vbus_detection to comply with the USB spec. If you enable it, the board
//         // has to support it or USB won't work at all. See docs on `vbus_detection` for details.
//         config.vbus_detection = true;
//         let driver = Driver::new_fs(usb_otg_fs, Irqs, pa12, pa11, &mut ep_out_buffer, config);
//         // let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
//         // config.manufacturer = Some("Embassy");
//         // config.product = Some("USB-serial example");
//         // config.serial_number = Some("12345678");
//         // // Create embassy-usb DeviceBuilder using the driver and config.
//         // // It needs some buffers for building the descriptors.
//         //
//         // let mut builder = {
//         //     static CONFIG_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
//         //     static BOS_DESCRIPTOR: StaticCell<[u8; 256]> = StaticCell::new();
//         //     static CONTROL_BUF: StaticCell<[u8; 64]> = StaticCell::new();
//         //
//         //     let builder = embassy_usb::Builder::new(
//         //         driver,
//         //         config,
//         //         CONFIG_DESCRIPTOR.init([0; 256]),
//         //         BOS_DESCRIPTOR.init([0; 256]),
//         //         &mut [], // no msos descriptors
//         //         CONTROL_BUF.init([0; 64]),
//         //     );
//         //     builder
//         // };
//         //
//         // // Create classes on the builder.
//         // let mut class = {
//         //     static STATE: StaticCell<State> = StaticCell::new();
//         //     let state = STATE.init(State::new());
//         //     CdcAcmClass::new(&mut builder, state, 64)
//         // };
//         // Create embassy-usb Config
//         let mut config = embassy_usb::Config::new(0xc0de, 0xcafe);
//         config.max_packet_size_0 = 64;
//         config.manufacturer = Some("Embassy");
//         config.product = Some("USB-serial example");
//         config.serial_number = Some("12345678");

//         // Create embassy-usb DeviceBuilder using the driver and config.
//         // It needs some buffers for building the descriptors.
//         let mut config_descriptor = [0; 256];
//         let mut bos_descriptor = [0; 256];
//         let mut control_buf = [0; 64];

//         let mut state = State::new();

//         let mut builder = Builder::new(
//             driver,
//             config,
//             &mut config_descriptor,
//             &mut bos_descriptor,
//             &mut [], // no msos descriptors
//             &mut control_buf,
//         );

//         // Create classes on the builder.
//         let mut class = CdcAcmClass::new(&mut builder, &mut state, 64);

//         // Build the builder.
//         let mut usb = builder.build();

//         // // Run the USB device.
//         // info!("Running?");
//         // spawner.spawn(usb_runner_task(usb)).expect("TODO: panic message");
//         // info!("Running!");
//         // // Do stuff with the class!
//         // loop {
//         //     class.wait_connection().await;
//         //     info!("Connected");
//         //     let _ = echo(&mut class).await;
//         //     info!("Disconnected");
//         // }
//         // Run the USB device.
//         let usb_fut = usb.run();

//         // Do stuff with the class!
//         let echo_fut = async {
//             loop {
//                 class.wait_connection().await;
//                 info!("Connected");
//                 let _ = echo(&mut class).await;
//                 info!("Disconnected");
//             }
//         };

//         // Run everything concurrently.
//         // If we had made everything `'static` above instead, we could do this using separate tasks instead.
//         join(usb_fut, echo_fut).await;
//     }
// }

// struct Disconnected {}

// impl From<EndpointError> for Disconnected {
//     fn from(val: EndpointError) -> Self {
//         match val {
//             EndpointError::BufferOverflow => panic!("Buffer overflow"),
//             EndpointError::Disabled => Disconnected {},
//         }
//     }
// }

// async fn echo<'d, T: Instance + 'd>(class: &mut CdcAcmClass<'d, Driver<'d, T>>) -> Result<(), Disconnected> {
//     let mut buf = [0; 64];
//     loop {
//         let n = class.read_packet(&mut buf).await?;
//         let data = &buf[..n];
//         info!("data: {:x}", data);
//         class.write_packet(data).await?;
//     }
// }

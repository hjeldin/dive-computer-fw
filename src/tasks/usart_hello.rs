// use cortex_m::prelude::_embedded_hal_serial_Read;
// use defmt::info;
// use embassy_stm32::{bind_interrupts, peripherals, usart, Peri};
// use embassy_stm32::peripherals::{DMA2_CH2, DMA2_CH3, DMA2_CH6, DMA2_CH7, PB6, PB7, PC7, USART1};
// use embassy_time::Timer;
// use heapless::String;
//
// // bind_interrupts!(struct Irqs {
// //     USART1 => usart::InterruptHandler<peripherals::USART1>;
// // });
//
// ///
// /// This task will be used to handle the communication between the computer and
// /// external sensors (for example oxygen sensors).
// /// Below is an example of communication with external sensors:
// ///
//
//
// fn bytes_to_ascii(bytes: &[u8]) -> String<64> {
//     bytes.iter()
//         .map(|b| if *b <= 127 {
//             char::from_u32(*b as u32).unwrap_or_default()
//         } else {
//             '?'
//         })
//         .collect::<String<64>>()
// }
// #[embassy_executor::task]
// pub async fn usart_hello(
//     usart_peri: Peri<'static, USART1>,
//     usart_tx: Peri<'static, PB6>,
//     usart_rx: Peri<'static, PB7>,
//     usart_rx_dma: Peri<'static, DMA2_CH6>,
//     usart_tx_dma: Peri<'static, DMA2_CH7>,
// ) {
//     let mut config = usart::Config::default();
//     config.baudrate = 115200;
//     config.data_bits = usart::DataBits::DataBits8;
//     config.parity = usart::Parity::ParityNone;
//     config.stop_bits = usart::StopBits::STOP1;
//     let mut usart_device = usart::Uart::new(
//         usart_peri,
//         usart_rx,
//         usart_tx,
//         Irqs,
//         usart_rx_dma,
//         usart_tx_dma,
//         config
//     ).unwrap();
//
//     loop {
//         let mut buf: [u8; 16] = [0; 16];
//         // usart_device.blocking_write("Hello World!\r\n".as_bytes()).unwrap();
//         match usart_device.read_until_idle(&mut buf).await {
//             Ok(n) => {
//                 info!("STM32 received ({}): {:?}", n, &buf[..n]);
//             }
//             Err(e) => {
//                 info!("Error: {:?}", e);
//             }
//         }
//         info!("{:?}", bytes_to_ascii(&buf[..]).as_str());
//         info!("USART Hello World!");
//
//         Timer::after_millis(500).await;
//     }
// }
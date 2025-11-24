//! Embassy task to periodically request air quality sensor data using the dive-computer-proto protocol
use defmt::info;
use embassy_stm32::{bind_interrupts, peripherals, usart, Peri};
use embassy_stm32::peripherals::{DMA2_CH6, DMA2_CH7, PB6, PB7, USART1};
use embassy_time::Timer;
use dive_computer_proto::commands::Command;
use dive_computer_proto::protocol::{Message, MessageKind};
use dive_computer_proto::sensor::ReadingType;

bind_interrupts!(struct Irqs {
    USART1 => usart::InterruptHandler<peripherals::USART1>;
});

#[embassy_executor::task]
pub async fn air_quality_request_task(
    usart_peri: Peri<'static, USART1>,
    usart_tx: Peri<'static, PB6>,
    usart_rx: Peri<'static, PB7>,
    usart_rx_dma: Peri<'static, DMA2_CH6>,
    usart_tx_dma: Peri<'static, DMA2_CH7>,
    air_quality_sensor_id: u16,
) {
    let mut config = usart::Config::default();
    config.baudrate = 9600;
    config.data_bits = usart::DataBits::DataBits8;
    config.parity = usart::Parity::ParityNone;
    config.stop_bits = usart::StopBits::STOP1;
    let mut usart_device = usart::Uart::new(
        usart_peri,
        usart_rx,
        usart_tx,
        Irqs,
        usart_rx_dma,
        usart_tx_dma,
        config
    ).unwrap();

    let sequence: u16;
    let rx_buf = [0xff; 16];
    loop {
        usart_device.write(&rx_buf).await.unwrap();
        // usart_device.read_until_idle(&mut rx_buf).await.unwrap();
        // info!("USART received: {:?}", &rx_buf[..]);
        Timer::after_millis(1000).await;
        continue;

        // Construct the ReadSensor command for air quality
        let command = Command::ReadSensor {
            sensor_id: air_quality_sensor_id,
            reading_type: ReadingType::Battery as u8 + 1, // Use the same convention as device
        };
        // Wrap in protocol message
        let msg = Message::new(MessageKind::Command, sequence, command).unwrap();
        if let Ok((size, buf)) = msg.serialize() {
            match usart_device.write(&buf[..size]).await {
                Ok(_) => info!("Sent air quality ReadSensor command (seq={})", sequence),
                Err(e) => info!("USART write error: {:?}", e),
            }
        } else {
            info!("Failed to serialize air quality command");
        }
        // Try to read a response from the sensor
        match usart_device.read_until_idle(&mut rx_buf).await {
            Ok(n) => {
                if let Ok(response_msg) = <Message<dive_computer_proto::commands::Response> as core::convert::TryFrom<&[u8]>>::try_from(&rx_buf[..n]) {
                    info!("Received response: {:?}", response_msg);
                    if let Some(dive_computer_proto::commands::ResponsePayload::SensorData { sensor_id, reading_type, value }) = response_msg.payload.payload {
                        info!("Air quality sensor {} reading_type {} value {}", sensor_id, reading_type, value);
                    }
                } else {
                    info!("Received non-protocol or invalid response");
                }
            }
            Err(e) => {
                info!("USART read error: {:?}", e);
            }
        }
        sequence = sequence.wrapping_add(1);
        Timer::after_millis(1000).await;
    }
}

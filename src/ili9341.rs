use embassy_time::Timer;
use ili9341regs::LcdOrientation;

use crate::spidriver::SPIDriver;

mod ili9341regs {
    macro_rules! mcpregs {
        ($($name:ident : $val:expr),* $(,)?) => {
            $(
                pub const $name: u8 = $val;
            )*

            pub fn regname(reg: u8) -> &'static str {
                match reg {
                    $(
                        $val => stringify!($name),
                    )*
                    _ => panic!("bad reg"),
                }
            }
        }
    }

    mcpregs! {
        SLEEP_OUT: 0x11,
        POWER_CONTROL_B: 0x1,
        POWER_ON_SEQ_CONTROL: 0xED,
        DRIVER_TIMING_CONTROL_A: 0xE8,
        POWER_CONTROL_A: 0xCB,
        PUMP_RATIO_CONTROL: 0xF7,
        DRIVER_TIMING_CONTROL_B: 0xEA,
        POWER_CONTROL_1: 0xC0,
        POWER_CONTROL_2: 0xC1,
        VCOM_CONTROL_1: 0xC5,
        VCOM_CONTROL_2: 0xC7,
        PIXEL_FORMAT_SET: 0x3A,
        MEMORY_ACCESS_CONTROL: 0x36,
        FRAME_CONTROL_NORMAL_MODE: 0xB1,
        DISPLAY_FUNCTION_CONTROL: 0xB6,
        SET_TEAR_SCANLINE: 0x44,
        DISPLAY_ON: 0x29,
        DISPLAY_OFF: 0x28,
        ENABLE_3G: 0xF2,
        GAMMA_SET: 0x26,
        POSITIVE_GAMMA_CORRECTION: 0xE0,
        NEGATIVE_GAMMA_CORRECTION: 0xE1,
    }

    pub enum LcdOrientation {
        Rotate0,
        Rotate90,
        Rotate180,
        Rotate270,
    }
}

pub struct ILI9341<'a> {
    driver: SPIDriver<'a>,
    orientation: LcdOrientation,
}

impl<'a> ILI9341<'a> {
    pub fn new(device: SPIDriver<'a>, lcd_orientation: LcdOrientation) -> Self {
        ILI9341 {
            driver: device,
            orientation: lcd_orientation,
        }
    }

    fn memory_access_control_value(&self) -> u8 {
        let orientation = match self.orientation {
            LcdOrientation::Rotate0 => 0b00000000,
            LcdOrientation::Rotate90 => 0b01100000,
            LcdOrientation::Rotate180 => 0b11000000,
            LcdOrientation::Rotate270 => 0b10100000,
        };
        orientation | 0b00001000
    }


    pub async fn init(&self) {
        self.driver.reset().await;
        // self.driver.write_command(&[ili9341regs::SLEEP_OUT]).await;
        // self.driver.write_command(&[0x21]).await;
        // self.driver
        //     .write_command(&[ili9341regs::POWER_CONTROL_B])
        //     .await;
        // self.driver.write_data(&[0x00, 0xC1, 0x30]).await;
        // self.driver
        //     .write_command(&[ili9341regs::POWER_ON_SEQ_CONTROL])
        //     .await;
        // self.driver.write_data(&[0x64, 0x03, 0x12, 0x81]).await;
        // self.driver
        //     .write_command(&[ili9341regs::DRIVER_TIMING_CONTROL_A])
        //     .await;
        // self.driver.write_data(&[0x85, 0x00, 0x79]).await;
        // self.driver
        //     .write_command(&[ili9341regs::POWER_CONTROL_A])
        //     .await;
        // self.driver
        //     .write_data(&[0x39, 0x2C, 0x00, 0x34, 0x02])
        //     .await;
        // self.driver
        //     .write_command(&[ili9341regs::PUMP_RATIO_CONTROL])
        //     .await;
        // self.driver.write_data(&[0x20]).await;
        // self.driver
        //     .write_command(&[ili9341regs::DRIVER_TIMING_CONTROL_B])
        //     .await;
        // self.driver.write_data(&[0x00, 0x00]).await;
        // self.driver
        //     .write_command(&[ili9341regs::POWER_CONTROL_1])
        //     .await;
        // self.driver.write_data(&[0x1D]).await;
        // self.driver
        //     .write_command(&[ili9341regs::POWER_CONTROL_2])
        //     .await;
        // self.driver.write_data(&[0x12]).await;
        // self.driver
        //     .write_command(&[ili9341regs::VCOM_CONTROL_1])
        //     .await;
        // self.driver.write_data(&[0x33, 0x3F]).await;
        // self.driver
        //     .write_command(&[ili9341regs::VCOM_CONTROL_2])
        //     .await;
        // self.driver.write_data(&[0x92]).await;
        // self.driver
        //     .write_command(&[ili9341regs::PIXEL_FORMAT_SET])
        //     .await;
        // self.driver.write_data(&[0x55]).await;
        // self.driver
        //     .write_command(&[ili9341regs::MEMORY_ACCESS_CONTROL])
        //     .await;
        // self.driver
        //     .write_data(&[self.memory_access_control_value()])
        //     .await;
        // self.driver
        //     .write_command(&[ili9341regs::FRAME_CONTROL_NORMAL_MODE])
        //     .await;
        // self.driver.write_data(&[0x00, 0x12]).await;
        // self.driver
        //     .write_command(&[ili9341regs::DISPLAY_FUNCTION_CONTROL])
        //     .await;
        // self.driver.write_data(&[0x0A, 0xA2]).await;
        // self.driver
        //     .write_command(&[ili9341regs::SET_TEAR_SCANLINE])
        //     .await;
        // self.driver.write_data(&[0x02]).await;
        // self.driver.write_command(&[ili9341regs::DISPLAY_ON]).await;

        // self.set_gamma().await;
        self.driver.sleep().await;
        self.driver.sleep().await;
        self.driver.sleep().await;
        self.driver.sleep().await;
        self.driver.sleep().await;
    }

    pub async fn set_gamma(&self) {
        self.driver.write_command(&[ili9341regs::ENABLE_3G]).await;
        self.driver.write_data(&[0x00]).await;
        self.driver.write_command(&[ili9341regs::GAMMA_SET]).await;
        self.driver.write_data(&[0x01]).await;
        self.driver
            .write_command(&[ili9341regs::POSITIVE_GAMMA_CORRECTION])
            .await;
        self.driver
            .write_data(&[
                0x0F, 0x22, 0x1c, 0x1b, 0x08, 0x0f, 0x48, 0xb8, 0x34, 0x05, 0x0c, 0x09, 0x0f, 0x07,
                0x00,
            ])
            .await;
        self.driver
            .write_command(&[ili9341regs::NEGATIVE_GAMMA_CORRECTION])
            .await;
        self.driver
            .write_data(&[
                0x00, 0x23, 0x24, 0x07, 0x10, 0x07, 0x38, 0x47, 0x4b, 0x0a, 0x13, 0x06, 0x30, 0x38,
                0x0f,
            ])
            .await;
    }
}

#[embassy_executor::task]
pub async fn screen_task(lcd: SPIDriver<'static>) {
    defmt::info!("ILI9341 task");
    let ili9341_lcd = ILI9341::new(lcd, LcdOrientation::Rotate0);
    defmt::info!("ILI9341 init");
    ili9341_lcd.init().await;
    defmt::info!("ILI9341 init done");
    loop {
        Timer::after_millis(1000).await;
    }
}

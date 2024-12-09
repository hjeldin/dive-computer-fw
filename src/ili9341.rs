use embassy_time::Timer;
use ili9341regs::{LcdOrientation, COLUMNS, PAGES};

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
        READ_ID: 0x04,
        READ_DISP_STATUS: 0x09,
        READ_DISP_POWER: 0x0A,
        READ_DISP_DIAG: 0x0F,
        SLEEP_OUT: 0x11,
        NORMAL_MODE_ON: 0x13,
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
        COLUMN_ADDRESS_SET: 0x2A,
        PAGE_ADDRESS_SET: 0x2B,
        MEMORY_WRITE: 0x2C,
    }

    pub enum LcdOrientation {
        Rotate0,
        Rotate90,
        Rotate180,
        Rotate270,
    }

    pub const COLUMNS: u16 = 240;
    pub const PAGES: u16 = 320;
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

    pub fn size(&self) -> (u16, u16) {
        match self.orientation {
            LcdOrientation::Rotate0 | LcdOrientation::Rotate180 => (COLUMNS, PAGES),
            LcdOrientation::Rotate90 | LcdOrientation::Rotate270 => (PAGES, COLUMNS),
        }
    }

    pub(crate) fn u16_to_bytes(val: u16) -> (u8, u8) {
        ((val >> 8) as u8, (val & 0xff) as u8)
    }

    pub(crate) async fn set_window(
        &mut self,
        x0: u16,
        y0: u16,
        x1: u16,
        y1: u16,
    ) {
        let c1 = x1.saturating_sub(1).max(x0);
        let p1 = y1.saturating_sub(1).max(y0);
        let (c0h, c0l) = Self::u16_to_bytes(x0);
        let (c1h, c1l) = Self::u16_to_bytes(c1);
        let (p0h, p0l) = Self::u16_to_bytes(y0);
        let (p1h, p1l) = Self::u16_to_bytes(p1);

        self.driver.write_command(&[ili9341regs::COLUMN_ADDRESS_SET]).await;
        self.driver.write_data(&[c0h, c0l, c1h, c1l]).await;

        self.driver.write_command(&[ili9341regs::PAGE_ADDRESS_SET]).await;
        self.driver.write_data(&[p0h, p0l, p1h, p1l]).await;

        self.driver.write_command(&[ili9341regs::MEMORY_WRITE]).await;
    }

    pub async fn clear(&mut self, color: u16) {
        let (w, h) = self.size();
        self.fill_rect(0, 0, w, h, color).await;
    }

    pub fn color_buffer<const N: usize>(color: u16) -> [u8; N] {
        let (h, l) = Self::u16_to_bytes(color);
        core::array::from_fn(|i| if i % 2 == 0 { h } else { l })
    }

    /// Draw filled rect or line (when width or height set to 1)
    pub async fn fill_rect(
        &mut self,
        x: u16,
        y: u16,
        w: u16,
        h: u16,
        color: u16,
    ) {
        self.set_window(x, y, x + w, y + h).await;
        self.driver.enable_write_data().await;

        // spi send optimization
        // slight buffer overflow seems ok
        let chunk = Self::color_buffer::<32>(color);
        for _ in 0..(w as u32 * h as u32).div_ceil(16) {
            self.driver.write_data_continue(&chunk).await;
        }
    }


    pub async fn init(&self) {
        self.driver.soft_reset().await;
        Timer::after_millis(100).await;

        self.driver.write_command(&[0xef]).await;
        self.driver.write_data(&[0x03, 0x80, 0x02]).await;

        self.driver.write_command(&[0xcf]).await;
        self.driver.write_data(&[0x00, 0xc1, 0x30]).await;


        self.driver.write_command(&[0xed]).await;
        self.driver.write_data(&[0x64, 0x03, 0x12, 0x81]).await;

        self.driver.write_command(&[0xe8]).await;
        self.driver.write_data(&[0x85, 0x00, 0x78]).await;

        self.driver.write_command(&[0xcb]).await;
        self.driver.write_data(&[0x39, 0x2c, 0x00, 0x34, 0x02]).await;

        self.driver.write_command(&[0xf7]).await;
        self.driver.write_data(&[0x20]).await;

        self.driver.write_command(&[0xea]).await;
        self.driver.write_data(&[0x00, 0x00]).await;

        self.driver.write_command(&[0xc0]).await;
        self.driver.write_data(&[0x23]).await;

        self.driver.write_command(&[0xc1]).await;
        self.driver.write_data(&[0x10]).await;

        self.driver.write_command(&[0xc5]).await;
        self.driver.write_data(&[0x3e, 0x28]).await;

        self.driver.write_command(&[0xc7]).await;
        self.driver.write_data(&[0x86]).await;

        self.driver.write_command(&[0x36]).await;
        self.driver.write_data(&[0x48]).await;

        self.driver.write_command(&[0x37]).await;
        self.driver.write_data(&[0x00]).await;

        self.driver.write_command(&[0x3a]).await;
        self.driver.write_data(&[0x55]).await;

        self.driver.write_command(&[0xb1]).await;
        self.driver.write_data(&[0x00, 0x18]).await;

        self.driver.write_command(&[0xb6]).await;
        self.driver.write_data(&[0x08, 0x82, 0x27]).await;

        self.driver.write_command(&[0xf2]).await;
        self.driver.write_data(&[0x00]).await;

        self.driver.write_command(&[0x26]).await;
        self.driver.write_data(&[0x01]).await;

        self.driver.write_command(&[0xe0]).await;
        self.driver.write_data(&[0x0f, 0x31, 0x2b, 0x0c, 0x0e, 0x08, 0x4e, 0xf1, 0x37, 0x07, 0x10, 0x03, 0x0e, 0x09, 0x00]).await;

        self.driver.write_command(&[0xe1]).await;
        self.driver.write_data(&[0x00, 0x0e, 0x14, 0x03, 0x11, 0x07, 0x31, 0xc1, 0x48, 0x08, 0x0f, 0x0c, 0x31, 0x36, 0x0f]).await;

        self.driver.write_command(&[0x11]).await;

        Timer::after_millis(120).await;

        self.driver.write_command(&[0x29]).await;

        Timer::after_millis(120).await;

        // self.driver.write_command(&[0xd9]).await;
        // self.driver.write_data(&[0x10]).await;

        // self.driver.write_command(&[0x0A]).await;
        // self.driver.write_data(&[0x00]).await;

        // self.driver.write_command(&[0xd9]).await;







        // self.driver.write_command(&[ili9341regs::SLEEP_OUT]).await;

        // self.driver.write_command(&[ili9341regs::DISPLAY_OFF]).await;

        // self.driver.write_command(&[ili9341regs::DISPLAY_ON]).await;

        // self.driver.write_command(&[ili9341regs::NORMAL_MODE_ON]).await;

        // let mut id = [0u8; 4];
        // self.driver.write_command(&[ili9341regs::READ_ID]).await;
        // Timer::after_millis(50).await;
        // self.driver.read_bytes(&mut id).await;

        // defmt::info!("ID: {:x}", id);

        // self.driver.write_command(&[ili9341regs::READ_DISP_STATUS]).await;
        // Timer::after_millis(50).await;
        // self.driver.read_bytes(&mut id).await;

        // defmt::info!("DISP STATUS: {:x}", id);

        // self.driver.write_command(&[ili9341regs::READ_DISP_POWER]).await;
        // Timer::after_millis(50).await;
        // self.driver.read_bytes(&mut id).await;

        // defmt::info!("DISP POWER: {:x}", id);

        // self.driver.write_command(&[ili9341regs::READ_DISP_DIAG]).await;
        // Timer::after_millis(50).await;
        // self.driver.read_bytes(&mut id).await;

        // defmt::info!("DISP DIAG: {:x}", id);

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
    let mut ili9341_lcd = ILI9341::new(lcd, LcdOrientation::Rotate0);
    defmt::info!("ILI9341 init");
    ili9341_lcd.init().await;
    defmt::info!("ILI9341 init done");
    ili9341_lcd.set_window(0, 0, 30, 30).await;
    Timer::after_millis(1000).await;
    ili9341_lcd.clear(0x0000).await;
    loop {
        Timer::after_millis(1000).await;
    }
}

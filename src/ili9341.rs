use core::{
    convert::{Infallible, TryInto},
    fmt::Write,
};

use defmt::info;
use embassy_stm32::{gpio::Output, mode::Async, pac::spi::Spi};
use embassy_sync::{blocking_mutex::raw::ThreadModeRawMutex, mutex::Mutex};
use embassy_time::{Instant, Timer};
use embedded_graphics::{
    iterator::pixel,
    primitives::{PrimitiveStyleBuilder, StyledDrawable},
};
use ili9341regs::{LcdOrientation, COLUMNS, PAGES};
use stm_graphics::scene_manager;

use crate::{spidriver::SPIDriver, LCD_REDRAW};
use heapless::String;

use embedded_graphics_core::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};

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
        POWER_CONTROL_B: 0xCF,
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
        VERTICAL_SCROLL_START_ADDR: 0x37,
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

    pub const CHAR_START: usize = 0x20;

    pub const FONT: &[u8] = include_bytes!("../assets/font.bin");
}

struct DrawCommand {
    x: u16,
    y: u16,
    // color: [u16; 128],
}

pub struct ILI9341 {
    driver: embassy_stm32::spi::Spi<'static, Async>,
    dc: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    orientation: LcdOrientation,
    // queue: [DrawCommand; 8]
}

impl ILI9341 {
    pub fn new(
        device: embassy_stm32::spi::Spi<'static, Async>,
        dc: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
        rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
        lcd_orientation: LcdOrientation,
    ) -> Self {
        ILI9341 {
            driver: device,
            orientation: lcd_orientation,
            // queue: core::array::from_fn(|_| DrawCommand {
            //     x: 0,
            //     y: 0,
            //     color: [0; 128]
            // }),
            dc,
            rst,
        }
    }

    fn memory_access_control_value(&self) -> u8 {
        let orientation = match self.orientation {
            LcdOrientation::Rotate0 => 0b00000000,
            LcdOrientation::Rotate90 => 0b00100000,
            LcdOrientation::Rotate180 => 0b11000000,
            LcdOrientation::Rotate270 => 0b10100000,
        };
        orientation | 0b00001000
    }

    pub fn inner_size(&self) -> (u16, u16) {
        match self.orientation {
            LcdOrientation::Rotate0 | LcdOrientation::Rotate180 => (COLUMNS, PAGES),
            LcdOrientation::Rotate90 | LcdOrientation::Rotate270 => (PAGES, COLUMNS),
        }
    }

    pub fn u16_to_bytes(val: u16) -> (u8, u8) {
        ((val >> 8) as u8, (val & 0xff) as u8)
    }

    pub fn set_window(&mut self, x0: u16, y0: u16, x1: u16, y1: u16) {
        let c1 = x1.saturating_sub(1).max(x0);
        let p1 = y1.saturating_sub(1).max(y0);
        let (c0h, c0l) = Self::u16_to_bytes(x0);
        let (c1h, c1l) = Self::u16_to_bytes(c1);
        let (p0h, p0l) = Self::u16_to_bytes(y0);
        let (p1h, p1l) = Self::u16_to_bytes(p1);

        self.write_command(&[ili9341regs::COLUMN_ADDRESS_SET]);
        self.driver.blocking_write(&[c0h, c0l, c1h, c1l]);

        self.write_command(&[ili9341regs::PAGE_ADDRESS_SET]);
        self.driver.blocking_write(&[p0h, p0l, p1h, p1l]);

        self.write_command(&[ili9341regs::MEMORY_WRITE]);
    }

    fn disable_write_data(&self) {
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_low();
    }

    pub fn clear(&mut self, color: u16) {
        let (w, h) = self.inner_size();
        self.fill_rect(0, 0, w, h, color);
    }

    pub fn color_buffer<const N: usize>(color: u16) -> [u8; N] {
        let (h, l) = Self::u16_to_bytes(color);
        core::array::from_fn(|i| if i % 2 == 0 { h } else { l })
    }

    /// Draw filled rect or line (when width or height set to 1)
    pub fn fill_rect(&mut self, x: u16, y: u16, w: u16, h: u16, color: u16) {
        self.set_window(x, y, x + w, y + h);
        self.enable_write_data();

        // spi send optimization
        // slight buffer overflow seems ok
        let chunk = Self::color_buffer::<32>(color);
        for _ in 0..(w as u32 * h as u32).div_ceil(16) {
            self.driver.blocking_write(&chunk);
        }
    }

    fn enable_write_data(&self) {
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_high();
    }

    pub fn draw_raw_iter(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, data: &[u8]) {
        self.set_window(x0, y0, x1, y1);
        self.enable_write_data();

        // for _ in 0..(x1 as u32 * y1 as u32).div_ceil(16) {
        self.driver.blocking_write(&data);
        // }
    }

    pub fn draw_sprite(&mut self, x: u16, y: u16, w: u16, h: u16, data: &[u8]) {
        self.set_window(x, y, x + w, y + h);
        self.driver.blocking_write(&data);
    }

    pub fn draw_text(
        &mut self,
        x: u16,
        y: u16,
        text: &str,
        fg_color: u16,
        bg_color: u16,
        scale: u16,
    ) {
        let mut cx = x;

        for c in text.bytes() {
            let offset = 8 * (c as usize - ili9341regs::CHAR_START);
            let data: [u8; 8] = ili9341regs::FONT[offset..offset + 8]
                .try_into()
                .unwrap_or_default();
            self.draw_character(cx, y, &data, fg_color, bg_color, scale);
            cx += scale * 8
        }
    }

    fn draw_character(
        &mut self,
        x: u16,
        y: u16,
        data: &[u8; 8],
        fg_color: u16,
        bg_color: u16,
        scale: u16,
    ) {
        // TODO make readable
        self.set_window(x, y, x + scale * 8, y + scale * 8);

        let (fgh, fgl) = u16_to_bytes(fg_color);
        let (bgh, bgl) = u16_to_bytes(bg_color);

        let mut buffer = [0; 16];
        let chunk = 8 / scale;

        self.enable_write_data();

        for row in 0..8 {
            for _ in 0..scale {
                for bit_offset in (0..scale).rev() {
                    let bit_start = bit_offset * chunk;
                    let bit_end = (bit_offset + 1) * chunk;
                    for i in bit_start..bit_end {
                        let (h, l) = if data[row] >> i & 1 == 0 {
                            (bgh, bgl)
                        } else {
                            (fgh, fgl)
                        };
                        let offset = 16 - ((i - bit_start + 1) * 2 * scale) as usize;
                        for b in 0..scale as usize {
                            buffer[offset + 2 * b + 1] = l;
                            buffer[offset + 2 * b] = h;
                        }
                    }
                    self.driver.blocking_write(&buffer);
                }
            }
        }
    }

    pub fn draw_line(&mut self, x1: i16, y1: i16, x2: i16, y2: i16, color: u16) {
        let dx = (x2 - x1).abs();
        let dy = -(y2 - y1).abs();
        let sx = if x1 < x2 { 1 } else { -1 };
        let sy = if y1 < y2 { 1 } else { -1 };
        let mut err = dx + dy;

        let mut x = x1;
        let mut y = y1;

        loop {
            self.draw_pixel(x as u16, y as u16, color);
            if x == x2 && y == y2 {
                break;
            }
            let e2 = 2 * err;
            if e2 >= dy {
                err += dy;
                x += sx;
            }
            if e2 <= dx {
                err += dx;
                y += sy;
            }
        }
    }

    pub fn draw_pixel(&mut self, x: u16, y: u16, color: u16) {
        self.set_window(x, y, x + 1, y + 1);
        self.driver.blocking_write(&Self::color_buffer::<2>(color));
    }

    fn write_command(&mut self, byte: &[u8]) {
        // let mut spi = self.try_lock().unwrap();?
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_low();
        self.driver.blocking_write(byte);
        dc.set_high();
    }

    pub async fn reset(&self) {
        let mut rst = self.rst.try_lock().unwrap();
        rst.set_high();
        Timer::after_millis(200).await;
        rst.set_low();
        Timer::after_millis(200).await;
        rst.set_high();
        Timer::after_millis(200).await;
    }

    pub async fn soft_reset(&mut self) {
        self.reset().await;
        self.write_command(&[0x01]);
        Timer::after_millis(200).await;
    }

    pub async fn init(&mut self) {
        self.soft_reset().await;
        Timer::after_millis(100).await;

        self.write_command(&[0xef]); // unknown
        self.driver.blocking_write(&[0x03u8, 0x80u8, 0x02u8]);

        self.write_command(&[ili9341regs::POWER_CONTROL_B]); // unknown
        self.driver.blocking_write(&[0x00u8, 0xc1u8, 0x30u8]);

        self.write_command(&[ili9341regs::POWER_ON_SEQ_CONTROL]);
        self.driver.blocking_write(&[0x64u8, 0x03u8, 0x12u8, 0x81]);

        self.write_command(&[ili9341regs::DRIVER_TIMING_CONTROL_A]);
        self.driver.blocking_write(&[0x85u8, 0x00u8, 0x78]);

        self.write_command(&[ili9341regs::POWER_CONTROL_A]);
        self.driver
            .blocking_write(&[0x39u8, 0x2cu8, 0x00u8, 0x34u8, 0x02]);

        self.write_command(&[ili9341regs::PUMP_RATIO_CONTROL]);
        self.driver.blocking_write(&[0x20u8]);

        self.write_command(&[ili9341regs::DRIVER_TIMING_CONTROL_B]);
        self.driver.blocking_write(&[0x00u8, 0x00]);

        self.write_command(&[ili9341regs::POWER_CONTROL_1]);
        self.driver.blocking_write(&[0x23u8]);

        self.write_command(&[ili9341regs::POWER_CONTROL_2]);
        self.driver.blocking_write(&[0x10u8]);

        self.write_command(&[ili9341regs::VCOM_CONTROL_1]);
        self.driver.blocking_write(&[0x3eu8, 0x28]);

        self.write_command(&[ili9341regs::VCOM_CONTROL_2]);
        self.driver.blocking_write(&[0x86u8]);

        self.write_command(&[ili9341regs::MEMORY_ACCESS_CONTROL]);
        self.driver
            .blocking_write(&[self.memory_access_control_value()]);

        self.write_command(&[ili9341regs::VERTICAL_SCROLL_START_ADDR]); // unknown
        self.driver.blocking_write(&[0x00u8]);

        self.write_command(&[ili9341regs::PIXEL_FORMAT_SET]);
        self.driver.blocking_write(&[0x55u8]);

        self.write_command(&[ili9341regs::FRAME_CONTROL_NORMAL_MODE]);
        self.driver.blocking_write(&[0x00u8, 0x18]);

        self.write_command(&[ili9341regs::DISPLAY_FUNCTION_CONTROL]);
        self.driver.blocking_write(&[0x08u8, 0x82u8, 0x27]);

        self.write_command(&[ili9341regs::ENABLE_3G]);
        self.driver.blocking_write(&[0x00u8]);

        self.write_command(&[ili9341regs::GAMMA_SET]);
        self.driver.blocking_write(&[0x01u8]);

        self.write_command(&[ili9341regs::POSITIVE_GAMMA_CORRECTION]);
        self.driver.blocking_write(&[
            0x0fu8, 0x31u8, 0x2bu8, 0x0cu8, 0x0eu8, 0x08u8, 0x4eu8, 0xf1u8, 0x37u8, 0x07u8, 0x10u8,
            0x03u8, 0x0eu8, 0x09u8, 0x00,
        ]);

        self.write_command(&[ili9341regs::NEGATIVE_GAMMA_CORRECTION]);
        self.driver.blocking_write(&[
            0x00u8, 0x0eu8, 0x14u8, 0x03u8, 0x11u8, 0x07u8, 0x31u8, 0xc1u8, 0x48u8, 0x08u8, 0x0fu8,
            0x0cu8, 0x31u8, 0x36u8, 0x0f,
        ]);

        self.write_command(&[ili9341regs::SLEEP_OUT]);

        Timer::after_millis(120).await;

        self.write_command(&[ili9341regs::DISPLAY_ON]);

        Timer::after_millis(120).await;
    }
}

impl OriginDimensions for ILI9341 {
    fn size(&self) -> Size {
        let size = self.inner_size();
        Size::new(size.0 as u32, size.1 as u32)
    }
}

impl DrawTarget for ILI9341 {
    type Color = Rgb565;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        for Pixel(point, color) in pixels {
            if self.bounding_box().contains(point) {
                let x = point.x as u16;
                let y = point.y as u16;
                let color = RawU16::from(color).into_inner();
                self.draw_pixel(x, y, color);
                // push_queue(&mut self, x, y, color);
            }
        }
        Ok(())
    }
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let drawable_area = area.intersection(&self.bounding_box());

        if let Some(drawable_bottom_right) = drawable_area.bottom_right() {
            let x0 = drawable_area.top_left.x as u16;
            let y0 = drawable_area.top_left.y as u16;
            let x1 = drawable_bottom_right.x as u16;
            let y1 = drawable_bottom_right.y as u16;

            const BUFFER_SIZE: usize = 64; // Buffer size (in bytes)
            let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
            let mut index = 0;

            let mut last_flush_x = x0; // Track the start of the current flush
            let mut last_flush_y = y0;

            // Iterate over drawable points and their colors
            let mut colors_iter = colors.into_iter();
            for y in y0..=y1 {
                for x in x0..=x1 {
                    // Get the next color
                    if let Some(color) = colors_iter.next() {
                        let raw = RawU16::from(color).into_inner();
                        let bytes = raw.to_be_bytes();

                        // Add pixel bytes to the buffer
                        buffer[index] = bytes[0];
                        buffer[index + 1] = bytes[1];
                        index += 2;

                        // If the buffer is full, flush it
                        if index == BUFFER_SIZE {
                            self.draw_raw_iter(last_flush_x, last_flush_y, x, y, &buffer[..index]);
                            index = 0; // Reset buffer
                            last_flush_x = x; // Start the next flush at the next pixel
                            last_flush_y = y;
                        }
                    }
                }

                // After finishing a row, reset `last_flush_x` to the start of the next row
                if index > 0 && y < y1 {
                    self.draw_raw_iter(last_flush_x, last_flush_y, x1, y, &buffer[..index]);
                    index = 0;
                    last_flush_x = x0;
                    last_flush_y = y + 1;
                }
            }

            // Flush any remaining bytes in the buffer
            if index > 0 {
                self.draw_raw_iter(last_flush_x, last_flush_y, x1, y1, &buffer[..index]);
            }

            Ok(())
        } else {
            // No pixels are drawable
            Ok(())
        }
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Infallible> {
        let _ = self.clear(RawU16::from(color).into_inner());
        return Ok(());
    }

    type Error = core::convert::Infallible;

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        self.fill_contiguous(area, core::iter::repeat(color))
    }
}

pub fn rgb_to_u16(r: u8, g: u8, b: u8) -> u16 {
    let rb = r >> 3;
    let gb = g >> 2;
    let bb = b >> 3;
    (rb as u16) << 11 | (gb as u16) << 5 | bb as u16
}

/// Combine RGB channels into 565 RGB format - as a (u8, u8) tuple
pub fn rgb_to_u8(r: u8, g: u8, b: u8) -> (u8, u8) {
    u16_to_bytes(rgb_to_u16(r, g, b))
}

pub(crate) fn u16_to_bytes(val: u16) -> (u8, u8) {
    ((val >> 8) as u8, (val & 0xff) as u8)
}

/// Create a single colored buffer of N/2 pixel length
pub fn color_buffer<const N: usize>(color: u16) -> [u8; N] {
    let (h, l) = u16_to_bytes(color);
    core::array::from_fn(|i| if i % 2 == 0 { h } else { l })
}

#[embassy_executor::task]
pub async fn screen_task(
    lcd: embassy_stm32::spi::Spi<'static, Async>,
    dc: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
) {
    defmt::info!("ILI9341 task");
    let mut ili9341_lcd = ILI9341::new(lcd, dc, rst, LcdOrientation::Rotate90);
    defmt::info!("ILI9341 init");
    ili9341_lcd.init().await;
    defmt::info!("ILI9341 init done");

    let mut scene_manager = scene_manager::SceneManager::new();
    scene_manager.add_scene(stm_graphics::startup::build_startup());

    let status_bar = stm_graphics::StatusBar::new(22, 32);
    let battery_indicator = stm_graphics::battery::BatteryIndicator::new(0xAA);

    // Timer::after_millis(1000).await;
    ili9341_lcd.clear(0xFFFF);
    Timer::after_millis(500).await;
    ili9341_lcd.clear(0x0000);
    Timer::after_millis(500).await;
    let mut lastrun = 0;
    let mut r = status_bar.draw(&mut ili9341_lcd);
    r = battery_indicator.draw(&mut ili9341_lcd);
    scene_manager.draw_active_scene(&mut ili9341_lcd);
    // let style = PrimitiveStyleBuilder::new()
    // // .stroke_color(Rgb565::WHITE)
    // // .stroke_width(3)
    // .fill_color(Rgb565::CSS_ORANGE_RED)
    // .build();
    // Rectangle::new(Point::new((320-150)/2, (240-150)/2), Size::new(150, 150))
    //     .draw_styled(&style, &mut ili9341_lcd)
    //     .unwrap();

    loop {
        // let state = crate::STATE.lock().await;

        // scene_manager.get_active_scene().unwrap().update(&state);
        // s.write_fmt(format_args!("Time: {:02}:{:02}", state.time[0], state.time[1])).unwrap();
        // ili9341_lcd.draw_text((320-160/2)/2, 240/2 - 80, s.as_str(), 0xFFFF, 0x0000, 1).await;
        // s.clear();
        // s.write_fmt(format_args!("Temperature: {:.2} C", state.temperature)).unwrap();
        // ili9341_lcd.draw_text((320-160/2)/2, 240/2 - 50, s.as_str(), 0xFFFF, 0x0000, 1).await;
        // ili9341_lcd.draw_line(0, 50, 320, 50, 0xff00).await;
        r = status_bar.draw(&mut ili9341_lcd);
        r = battery_indicator.draw(&mut ili9341_lcd);
        scene_manager.draw_active_scene(&mut ili9341_lcd);
        info!("Redraw: {}", Instant::now().as_millis() - lastrun);
        lastrun = Instant::now().as_millis();
        Timer::after_millis(33).await;
    }
}

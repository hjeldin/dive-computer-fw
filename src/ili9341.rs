use core::{
    convert::{Infallible, TryInto},
    mem,
    sync::atomic::{AtomicBool, Ordering},
};

use defmt::info;
use embassy_stm32::{gpio::Output, mode::Async, spi::mode::Master};
use embassy_sync::blocking_mutex::raw::ThreadModeRawMutex;
use embassy_sync::mutex::Mutex;
use embassy_sync::rwlock::RwLock;
use embassy_time::{with_timeout, Duration, Instant, Timer};
use ili9341regs::{LcdOrientation, COLUMNS, PAGES};
use stm_graphics::{scene_manager::SceneManager, Action, Date, DateTime, Time};

use crate::display_buffer::BufferedRegion;
use crate::{LCD_ENTER_ITEM, LCD_NEXT_ITEM, LCD_REFRESH, LCD_WAKE, LOW_POWER_MODE};
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
}

pub struct ILI9341 {
    driver: embassy_stm32::spi::Spi<'static, Async, Master>,
    dc: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    cs: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    orientation: LcdOrientation,
}

impl ILI9341 {
    pub fn new(
        device: embassy_stm32::spi::Spi<'static, Async, Master>,
        dc: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
        rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
        cs: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
        lcd_orientation: LcdOrientation,
    ) -> Self {
        ILI9341 {
            driver: device,
            orientation: lcd_orientation,
            dc,
            rst,
            cs,
        }
    }

    fn get_device(self) -> embassy_stm32::spi::Spi<'static, Async, Master> {
        self.driver
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
        let _ = self.driver.blocking_write(&[c0h, c0l, c1h, c1l]);

        self.write_command(&[ili9341regs::PAGE_ADDRESS_SET]);
        let _ = self.driver.blocking_write(&[p0h, p0l, p1h, p1l]);

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

        // let mut cs = self.cs.try_lock().unwrap();
        // cs.set_low();
        // spi send optimization
        // slight buffer overflow seems ok
        let chunk = Self::color_buffer::<32>(color);
        for _ in 0..(w as u32 * h as u32).div_ceil(16) {
            let _ = self.driver.blocking_write(&chunk);
        }
        // cs.set_high();
    }

    fn enable_write_data(&self) {
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_high();
    }

    pub fn draw_raw_iter(&mut self, x0: u16, y0: u16, x1: u16, y1: u16, data: &[u8]) {
        self.set_window(x0, y0, x1, y1);
        self.enable_write_data();
        let _ = self.driver.blocking_write(&data);
    }

    /// Write `bytes` (big-endian RGB565) as one horizontal run starting at `start`.
    fn write_run(&mut self, start: Point, bytes: &[u8]) {
        if bytes.is_empty() {
            return;
        }
        let (x, y) = (start.x as u16, start.y as u16);
        self.draw_raw_iter(x, y, x + (bytes.len() / 2) as u16, y + 1, bytes);
    }

    fn write_command(&mut self, byte: &[u8]) {
        let mut dc = self.dc.try_lock().unwrap();
        dc.set_low();
        let _ = self.driver.blocking_write(byte);
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
        // let mut cs = self.cs.try_lock().unwrap();
        // cs.set_low();

        self.write_command(&[0xef]); // unknown
        let _ = self.driver.blocking_write(&[0x03u8, 0x80u8, 0x02u8]);

        self.write_command(&[ili9341regs::POWER_CONTROL_B]); // unknown
        let _ = self.driver.blocking_write(&[0x00u8, 0xc1u8, 0x30u8]);

        self.write_command(&[ili9341regs::POWER_ON_SEQ_CONTROL]);
        let _ = self.driver.blocking_write(&[0x64u8, 0x03u8, 0x12u8, 0x81]);

        self.write_command(&[ili9341regs::DRIVER_TIMING_CONTROL_A]);
        let _ = self.driver.blocking_write(&[0x85u8, 0x00u8, 0x78]);

        self.write_command(&[ili9341regs::POWER_CONTROL_A]);
        let _ = self
            .driver
            .blocking_write(&[0x39u8, 0x2cu8, 0x00u8, 0x34u8, 0x02]);

        self.write_command(&[ili9341regs::PUMP_RATIO_CONTROL]);
        let _ = self.driver.blocking_write(&[0x20u8]);

        self.write_command(&[ili9341regs::DRIVER_TIMING_CONTROL_B]);
        let _ = self.driver.blocking_write(&[0x00u8, 0x00]);

        self.write_command(&[ili9341regs::POWER_CONTROL_1]);
        let _ = self.driver.blocking_write(&[0x23u8]);

        self.write_command(&[ili9341regs::POWER_CONTROL_2]);
        let _ = self.driver.blocking_write(&[0x10u8]);

        self.write_command(&[ili9341regs::VCOM_CONTROL_1]);
        let _ = self.driver.blocking_write(&[0x3eu8, 0x28]);

        self.write_command(&[ili9341regs::VCOM_CONTROL_2]);
        let _ = self.driver.blocking_write(&[0x86u8]);

        self.write_command(&[ili9341regs::MEMORY_ACCESS_CONTROL]);
        let _ = self
            .driver
            .blocking_write(&[self.memory_access_control_value()]);

        self.write_command(&[ili9341regs::VERTICAL_SCROLL_START_ADDR]); // unknown
        let _ = self.driver.blocking_write(&[0x00u8]);

        self.write_command(&[ili9341regs::PIXEL_FORMAT_SET]);
        let _ = self.driver.blocking_write(&[0x55u8]);

        self.write_command(&[ili9341regs::FRAME_CONTROL_NORMAL_MODE]);
        let _ = self.driver.blocking_write(&[0x00u8, 0x18]);

        self.write_command(&[ili9341regs::DISPLAY_FUNCTION_CONTROL]);
        let _ = self.driver.blocking_write(&[0x08u8, 0x82u8, 0x27]);

        self.write_command(&[ili9341regs::ENABLE_3G]);
        let _ = self.driver.blocking_write(&[0x00u8]);

        self.write_command(&[ili9341regs::GAMMA_SET]);
        let _ = self.driver.blocking_write(&[0x01u8]);

        self.write_command(&[ili9341regs::POSITIVE_GAMMA_CORRECTION]);
        let _ = self.driver.blocking_write(&[
            0x0fu8, 0x31u8, 0x2bu8, 0x0cu8, 0x0eu8, 0x08u8, 0x4eu8, 0xf1u8, 0x37u8, 0x07u8, 0x10u8,
            0x03u8, 0x0eu8, 0x09u8, 0x00,
        ]);

        self.write_command(&[ili9341regs::NEGATIVE_GAMMA_CORRECTION]);
        let _ = self.driver.blocking_write(&[
            0x00u8, 0x0eu8, 0x14u8, 0x03u8, 0x11u8, 0x07u8, 0x31u8, 0xc1u8, 0x48u8, 0x08u8, 0x0fu8,
            0x0cu8, 0x31u8, 0x36u8, 0x0f,
        ]);
        // cs.set_high();

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
        // Setting the window costs 3 commands, so neighbouring pixels on a
        // row (text glyphs, horizontal strokes) are collected and sent as
        // one run behind a single window.
        const RUN_PIXELS: usize = 64;
        let bounds = self.bounding_box();
        let mut run = [0u8; RUN_PIXELS * 2];
        let mut len = 0;
        let mut start = Point::zero();
        for Pixel(point, color) in pixels {
            if !bounds.contains(point) {
                continue;
            }
            let extends_run = len > 0
                && len < RUN_PIXELS
                && point.y == start.y
                && point.x == start.x + len as i32;
            if !extends_run {
                self.write_run(start, &run[..len * 2]);
                start = point;
                len = 0;
            }
            run[len * 2..len * 2 + 2].copy_from_slice(&RawU16::from(color).into_inner().to_be_bytes());
            len += 1;
        }
        self.write_run(start, &run[..len * 2]);
        Ok(())
    }
    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let drawable_area = area.intersection(&self.bounding_box());
        let Some(bottom_right) = drawable_area.bottom_right() else {
            // No pixels are drawable
            return Ok(());
        };

        // Set the window once for the whole rectangle; the controller then
        // advances through it on every pixel, so pixels can be streamed.
        // set_window takes an exclusive end.
        self.set_window(
            drawable_area.top_left.x as u16,
            drawable_area.top_left.y as u16,
            bottom_right.x as u16 + 1,
            bottom_right.y as u16 + 1,
        );
        self.enable_write_data();

        const BUFFER_SIZE: usize = 1024; // Buffer size (in bytes)
        let mut buffer: [u8; BUFFER_SIZE] = [0; BUFFER_SIZE];
        let mut index = 0;

        // `colors` covers the full `area`; skip pixels outside the screen.
        for (point, color) in area.points().zip(colors) {
            if !drawable_area.contains(point) {
                continue;
            }
            let bytes = RawU16::from(color).into_inner().to_be_bytes();
            buffer[index] = bytes[0];
            buffer[index + 1] = bytes[1];
            index += 2;

            if index == BUFFER_SIZE {
                let _ = self.driver.blocking_write(&buffer);
                index = 0;
            }
        }

        // Flush any remaining bytes in the buffer
        if index > 0 {
            let _ = self.driver.blocking_write(&buffer[..index]);
        }

        Ok(())
    }

    fn clear(&mut self, color: Self::Color) -> Result<(), Infallible> {
        let _ = self.clear(RawU16::from(color).into_inner());
        return Ok(());
    }

    type Error = core::convert::Infallible;

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let drawable_area = area.intersection(&self.bounding_box());
        if drawable_area.is_zero_sized() {
            return Ok(());
        }
        self.fill_rect(
            drawable_area.top_left.x as u16,
            drawable_area.top_left.y as u16,
            drawable_area.size.width as u16,
            drawable_area.size.height as u16,
            RawU16::from(color).into_inner(),
        );
        Ok(())
    }
}

/// Screen area of the compass in the dive scene; must match
/// `stm_graphics::dive::build_dive`. The compass clears and redraws its whole
/// area on every heading change, so it is drawn off-screen first.
const COMPASS_AREA: Rectangle = Rectangle::new(Point::new(200, 16), Size::new(120, 111));
const COMPASS_BYTES: usize = 120 * 111 * 2;

/// 26 KB does not fit beside everything else in SRAM1, so it lives in SRAM2.
/// memory.x declares SRAM2 as RAM2 but the linker places nothing there, so
/// the buffer takes it by address instead of through a linker section.
const SRAM2_START: usize = 0x1000_0000;
const SRAM2_SIZE: usize = 32 * 1024;
const _: () = assert!(COMPASS_BYTES <= SRAM2_SIZE);
static COMPASS_PIXELS_TAKEN: AtomicBool = AtomicBool::new(false);

/// The compass buffer, cleared to black. Panics if called twice.
fn take_compass_pixels() -> &'static mut [u8] {
    assert!(!COMPASS_PIXELS_TAKEN.swap(true, Ordering::Relaxed), "compass buffer taken twice");
    let pixels = SRAM2_START as *mut u8;
    // SAFETY: nothing is linked into SRAM2, the flag above hands this memory
    // out once, and it is zeroed before the slice is made
    unsafe {
        pixels.write_bytes(0, COMPASS_BYTES);
        core::slice::from_raw_parts_mut(pixels, COMPASS_BYTES)
    }
}

/// Value in whole tenths, the precision the screen shows.
fn tenths(value: f32) -> i32 {
    libm::roundf(value * 10.0) as i32
}

#[embassy_executor::task]
pub async fn screen_task(
    lcd: embassy_stm32::spi::Spi<'static, Async, Master>,
    dc: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    rst: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    cs: &'static Mutex<ThreadModeRawMutex, Output<'static>>,
    state: &'static RwLock<ThreadModeRawMutex, crate::state::State> 
) {
    cs.try_lock().unwrap().set_low();
    let mut ili9341_lcd = ILI9341::new(lcd, dc, rst, cs, LcdOrientation::Rotate90);
    ili9341_lcd.init().await;
    cs.try_lock().unwrap().set_high();
    let mut scene_manager = SceneManager::new();
    scene_manager.add_scene(stm_graphics::startup::build_startup());
    scene_manager.add_scene(stm_graphics::dive::build_dive());

    let mut status_bar = stm_graphics::StatusBar::new(22, 32);
    let mut battery_indicator = stm_graphics::battery::BatteryIndicator::new(0xAA);

    // Timer::after_millis(1000).await;
    cs.try_lock().unwrap().set_low();
    ili9341_lcd.clear(0xFFFF);
    Timer::after_millis(500).await;
    ili9341_lcd.clear(0x0000);
    // The panel is black now, matching the zeroed buffer
    let mut compass = BufferedRegion::new(COMPASS_AREA, take_compass_pixels());
    Timer::after_millis(500).await;
    let mut lastrun;
    let _ = status_bar.draw(&mut compass.target(&mut ili9341_lcd));
    let _ = battery_indicator.draw(&mut compass.target(&mut ili9341_lcd));
    let _ = scene_manager.draw_active_scene(&mut compass.target(&mut ili9341_lcd));
    compass.flush(&mut ili9341_lcd);
    cs.try_lock().unwrap().set_high();

    let mut gfx_state = stm_graphics::State {
        pressure: 12.0,
        temperature: 35.0,
        datetime: DateTime {
            date: Date {
                day: 12,
                month: 8,
                year: 2021,
            },
            time: Time::new(22, 35),
        },
        battery: 0xFF,
        uptime_ms: 0,
        dive_time_ms: 0,
        heading: 0.0,
        set_course: None,
    };

    loop {
        let (pressure, temperature, batt_voltage, heading) = {
            let state = state.read().await;
            (state.pressure, state.temperature, state.batt_voltage, state.mag_heading)
        };
        let mut refresh = LCD_REFRESH.swap(false, Ordering::Relaxed);
        if LCD_NEXT_ITEM.swap(false, Ordering::Relaxed) {
            scene_manager
                .get_active_scene()
                .unwrap()
                .select_next_toggleable();
            refresh = true;
        }

        if LCD_ENTER_ITEM.swap(false, Ordering::Relaxed) {
            let action = scene_manager
                .get_active_scene()
                .unwrap()
                .click_currently_toggled();
            if let Action::SwitchScene(index) = action {
                scene_manager.switch_scene(index);
            }
            refresh = true;
        }

        // Depth and temperature are shown to 0.1, so smaller changes are invisible
        if tenths(pressure) != tenths(gfx_state.pressure)
            || tenths(temperature) != tenths(gfx_state.temperature)
        {
            gfx_state.pressure = pressure;
            gfx_state.temperature = temperature;
            refresh = true;
        }

        // The compass shows whole degrees
        if libm::roundf(heading) as i32 != libm::roundf(gfx_state.heading) as i32 {
            gfx_state.heading = heading;
            refresh = true;
        }

        let battery_level = ((batt_voltage - 3.2) / (4.2 - 3.2) * 255.0) as u8;
        if battery_level != gfx_state.battery {
            gfx_state.battery = battery_level;
            battery_indicator.set_level(gfx_state.battery);
            cs.try_lock().unwrap().set_low();
            let _ = battery_indicator.draw(&mut compass.target(&mut ili9341_lcd));
            compass.flush(&mut ili9341_lcd);
            cs.try_lock().unwrap().set_high();
            info!("Battery level: {}", gfx_state.battery);
        }

        scene_manager.get_active_scene().unwrap().update(&gfx_state);
        if refresh {
            cs.try_lock().unwrap().set_low();
            let _ = scene_manager.draw_active_scene(&mut compass.target(&mut ili9341_lcd));
            compass.flush(&mut ili9341_lcd);
            cs.try_lock().unwrap().set_high();
        }
        lastrun = Instant::now().as_millis();
        // Sleep until a button, sensor or low power change wakes us.
        // The timeout is only a fallback so low power mode is never missed.
        let _ = with_timeout(Duration::from_secs(1), LCD_WAKE.wait()).await;
        if LOW_POWER_MODE.load(Ordering::Relaxed) == true {
            info!("[Display] Low power mode");
            break;
        }
    }

    let driver = ili9341_lcd.get_device();
    mem::drop(driver);

    info!("[Display] Exiting");
}

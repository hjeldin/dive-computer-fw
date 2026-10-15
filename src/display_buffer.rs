//! Off-screen buffer for one area of the screen.
//!
//! A component that clears its area and then draws on top would show every
//! step on the panel. Drawn into this buffer instead, the finished result
//! reaches the panel in one transfer.

use core::convert::Infallible;

use embedded_graphics_core::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};

use crate::ili9341::ILI9341;

pub struct BufferedRegion {
    area: Rectangle,
    /// Pixels in panel byte order (big-endian RGB565), row by row.
    pixels: &'static mut [u8],
    dirty: bool,
}

impl BufferedRegion {
    /// `pixels` holds 2 bytes per pixel of `area`. Its content must match the
    /// panel, e.g. all zero for a panel that was just cleared to black.
    pub fn new(area: Rectangle, pixels: &'static mut [u8]) -> Self {
        assert_eq!(pixels.len(), (area.size.width * area.size.height * 2) as usize);
        Self { area, pixels, dirty: false }
    }

    /// Draw target that writes pixels in this region to the buffer and all
    /// others to the panel. Call `flush` after drawing.
    pub fn target<'a>(&'a mut self, lcd: &'a mut ILI9341) -> Buffered<'a> {
        Buffered { lcd, region: self }
    }

    /// Send the buffer to the panel if anything was drawn into it.
    pub fn flush(&mut self, lcd: &mut ILI9341) {
        let Some(bottom_right) = self.area.bottom_right() else {
            return;
        };
        if !self.dirty {
            return;
        }
        // draw_raw_iter takes an exclusive end
        lcd.draw_raw_iter(
            self.area.top_left.x as u16,
            self.area.top_left.y as u16,
            bottom_right.x as u16 + 1,
            bottom_right.y as u16 + 1,
            self.pixels,
        );
        self.dirty = false;
    }

    fn index(&self, point: Point) -> usize {
        let x = (point.x - self.area.top_left.x) as usize;
        let y = (point.y - self.area.top_left.y) as usize;
        (y * self.area.size.width as usize + x) * 2
    }

    /// `point` must be inside the region.
    fn set(&mut self, point: Point, color: Rgb565) {
        let index = self.index(point);
        self.pixels[index..index + 2].copy_from_slice(&RawU16::from(color).into_inner().to_be_bytes());
        self.dirty = true;
    }

    /// `area` must be inside the region.
    fn fill(&mut self, area: &Rectangle, color: Rgb565) {
        let bytes = RawU16::from(color).into_inner().to_be_bytes();
        let row_bytes = area.size.width as usize * 2;
        let stride = self.area.size.width as usize * 2;
        let mut start = self.index(area.top_left);
        for _ in 0..area.size.height {
            for pixel in self.pixels[start..start + row_bytes].chunks_exact_mut(2) {
                pixel.copy_from_slice(&bytes);
            }
            start += stride;
        }
        self.dirty = true;
    }
}

pub struct Buffered<'a> {
    lcd: &'a mut ILI9341,
    region: &'a mut BufferedRegion,
}

impl OriginDimensions for Buffered<'_> {
    fn size(&self) -> Size {
        self.lcd.size()
    }
}

impl DrawTarget for Buffered<'_> {
    type Color = Rgb565;
    type Error = Infallible;

    fn draw_iter<I>(&mut self, pixels: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Pixel<Self::Color>>,
    {
        let region = &mut *self.region;
        self.lcd.draw_iter(pixels.into_iter().filter(|Pixel(point, color)| {
            if region.area.contains(*point) {
                region.set(*point, *color);
                false
            } else {
                true
            }
        }))
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        let inside = area.intersection(&self.region.area);
        if inside.is_zero_sized() {
            return self.lcd.fill_contiguous(area, colors);
        }
        if inside == *area {
            for (point, color) in area.points().zip(colors) {
                self.region.set(point, color);
            }
            return Ok(());
        }
        // Straddles the region's edge: rare, so take the per-pixel path
        self.draw_iter(area.points().zip(colors).map(|(point, color)| Pixel(point, color)))
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        let inside = area.intersection(&self.region.area);
        if inside.is_zero_sized() {
            return self.lcd.fill_solid(area, color);
        }
        self.region.fill(&inside, color);
        for band in around(area, &inside) {
            self.lcd.fill_solid(&band, color)?;
        }
        Ok(())
    }
}

/// The parts of `outer` not covered by `inner`, which must lie inside `outer`:
/// full-width bands above and below, and the sides beside `inner`.
fn around(outer: &Rectangle, inner: &Rectangle) -> impl Iterator<Item = Rectangle> {
    let (o0, i0) = (outer.top_left, inner.top_left);
    let o1 = o0 + outer.size;
    let i1 = i0 + inner.size;
    let rect = |x0: i32, y0: i32, x1: i32, y1: i32| {
        Rectangle::new(Point::new(x0, y0), Size::new((x1 - x0) as u32, (y1 - y0) as u32))
    };
    [
        rect(o0.x, o0.y, o1.x, i0.y),
        rect(o0.x, i1.y, o1.x, o1.y),
        rect(o0.x, i0.y, i0.x, i1.y),
        rect(i1.x, i0.y, o1.x, i1.y),
    ]
    .into_iter()
    .filter(|band| !band.is_zero_sized())
}

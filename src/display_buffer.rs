//! Off-screen buffers for areas of the screen.
//!
//! A component that clears its area and then draws on top would show every
//! step on the panel. Drawn into a buffer instead, the finished result
//! reaches the panel in one transfer.
//!
//! The buffers hold 4-bit indices into a per-region palette of 16 colours
//! instead of RGB565, a quarter of the size. Components here use only a
//! handful of colours and no anti-aliasing, so 16 is plenty.

use core::convert::Infallible;

use embassy_futures::join::join;
use embedded_graphics_core::{
    pixelcolor::{raw::RawU16, Rgb565},
    prelude::*,
    primitives::Rectangle,
};

use crate::ili9341::ILI9341;

const PALETTE_SIZE: usize = 16;
/// Pixels expanded to RGB565 per DMA transfer during a flush.
const CHUNK_PIXELS: usize = 512;

pub struct Region {
    area: Rectangle,
    /// Two pixels per byte as palette indices, row by row; the first pixel
    /// of each pair is in the high nibble.
    pixels: &'static mut [u8],
    palette: [Rgb565; PALETTE_SIZE],
    colors: usize,
    dirty: bool,
    palette_full_logged: bool,
}

impl Region {
    /// Bytes of buffer a region of `size` needs.
    pub const fn bytes(size: Size) -> usize {
        (size.width * size.height).div_ceil(2) as usize
    }

    /// `pixels` holds `Region::bytes(area.size)` bytes, all zero: the region
    /// starts black, like a panel that was just cleared to black.
    pub fn new(area: Rectangle, pixels: &'static mut [u8]) -> Self {
        assert_eq!(pixels.len(), Self::bytes(area.size));
        Self {
            area,
            pixels,
            palette: [Rgb565::BLACK; PALETTE_SIZE],
            colors: 1,
            dirty: false,
            palette_full_logged: false,
        }
    }

    fn color_index(&mut self, color: Rgb565) -> u8 {
        if let Some(index) = self.palette[..self.colors].iter().position(|&c| c == color) {
            return index as u8;
        }
        if self.colors < PALETTE_SIZE {
            self.palette[self.colors] = color;
            self.colors += 1;
            return (self.colors - 1) as u8;
        }
        if !self.palette_full_logged {
            defmt::warn!("Display buffer palette full, using the nearest colour");
            self.palette_full_logged = true;
        }
        let distance = |c: &Rgb565| {
            let d = |a: u8, b: u8| (a as i32 - b as i32).pow(2);
            d(c.r(), color.r()) + d(c.g(), color.g()) + d(c.b(), color.b())
        };
        let nearest = self.palette.iter().enumerate().min_by_key(|(_, c)| distance(c));
        nearest.map_or(0, |(index, _)| index as u8)
    }

    /// Pixel number of `point`, which must be inside the region.
    fn pixel_number(&self, point: Point) -> usize {
        let x = (point.x - self.area.top_left.x) as usize;
        let y = (point.y - self.area.top_left.y) as usize;
        y * self.area.size.width as usize + x
    }

    fn set_index(&mut self, pixel: usize, index: u8) {
        let byte = &mut self.pixels[pixel / 2];
        *byte = if pixel % 2 == 0 {
            (*byte & 0x0F) | (index << 4)
        } else {
            (*byte & 0xF0) | index
        };
    }

    /// `point` must be inside the region.
    fn set(&mut self, point: Point, color: Rgb565) {
        let index = self.color_index(color);
        self.set_index(self.pixel_number(point), index);
        self.dirty = true;
    }

    /// `area` must be inside the region.
    fn fill(&mut self, area: &Rectangle, color: Rgb565) {
        self.dirty = true;
        if *area == self.area {
            // Every pixel changes, so start a fresh palette: colours no
            // longer on screen free their slots
            self.palette[0] = color;
            self.colors = 1;
            self.pixels.fill(0);
            return;
        }
        let index = self.color_index(color);
        let stride = self.area.size.width as usize;
        let first = self.pixel_number(area.top_left);
        for row in 0..area.size.height as usize {
            let start = first + row * stride;
            for pixel in start..start + area.size.width as usize {
                self.set_index(pixel, index);
            }
        }
    }

    /// Expand pixels from `first` on into RGB565 bytes in `out`; returns the
    /// number of bytes written, 0 once all pixels are done.
    fn expand(&self, colors: &[[u8; 2]; PALETTE_SIZE], first: usize, out: &mut [u8]) -> usize {
        let total = (self.area.size.width * self.area.size.height) as usize;
        let count = (out.len() / 2).min(total - first);
        for (offset, bytes) in out[..count * 2].chunks_exact_mut(2).enumerate() {
            let pixel = first + offset;
            let byte = self.pixels[pixel / 2];
            let index = if pixel % 2 == 0 { byte >> 4 } else { byte & 0x0F };
            bytes.copy_from_slice(&colors[index as usize]);
        }
        count * 2
    }

    /// Send the region to the panel if anything was drawn into it. Each
    /// chunk is expanded to RGB565 while the previous one goes out by DMA.
    async fn flush(&mut self, lcd: &mut ILI9341) {
        let Some(bottom_right) = self.area.bottom_right() else {
            return;
        };
        if !self.dirty {
            return;
        }
        let colors = self.palette.map(|c| RawU16::from(c).into_inner().to_be_bytes());
        // begin_write takes an exclusive end
        lcd.begin_write(
            self.area.top_left.x as u16,
            self.area.top_left.y as u16,
            bottom_right.x as u16 + 1,
            bottom_right.y as u16 + 1,
        );

        let [mut sending, mut filling] = [[0u8; CHUNK_PIXELS * 2]; 2];
        let mut len = self.expand(&colors, 0, &mut sending);
        let mut next = len / 2;
        while len > 0 {
            let region = &*self;
            let (_, next_len) = join(lcd.write_dma(&sending[..len]), async {
                region.expand(&colors, next, &mut filling)
            })
            .await;
            core::mem::swap(&mut sending, &mut filling);
            next += next_len / 2;
            len = next_len;
        }
        self.dirty = false;
    }
}

/// Several regions; pixels outside all of them go straight to the panel.
pub struct BufferedRegions<const N: usize> {
    regions: [Region; N],
}

impl<const N: usize> BufferedRegions<N> {
    /// The regions must not overlap.
    pub fn new(regions: [Region; N]) -> Self {
        Self { regions }
    }

    /// Draw target that writes pixels in a region to its buffer and all
    /// others to the panel. Call `flush` after drawing.
    pub fn target<'a>(&'a mut self, lcd: &'a mut ILI9341) -> Buffered<'a> {
        Buffered { lcd, regions: &mut self.regions }
    }

    /// Send every region that was drawn into to the panel.
    pub async fn flush(&mut self, lcd: &mut ILI9341) {
        for region in &mut self.regions {
            region.flush(lcd).await;
        }
    }
}

pub struct Buffered<'a> {
    lcd: &'a mut ILI9341,
    regions: &'a mut [Region],
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
        let regions = &mut *self.regions;
        self.lcd.draw_iter(pixels.into_iter().filter(|Pixel(point, color)| {
            match regions.iter_mut().find(|region| region.area.contains(*point)) {
                Some(region) => {
                    region.set(*point, *color);
                    false
                }
                None => true,
            }
        }))
    }

    fn fill_contiguous<I>(&mut self, area: &Rectangle, colors: I) -> Result<(), Self::Error>
    where
        I: IntoIterator<Item = Self::Color>,
    {
        if self.regions.iter().all(|region| area.intersection(&region.area).is_zero_sized()) {
            return self.lcd.fill_contiguous(area, colors);
        }
        if let Some(region) = self
            .regions
            .iter_mut()
            .find(|region| area.intersection(&region.area) == *area)
        {
            for (point, color) in area.points().zip(colors) {
                region.set(point, color);
            }
            return Ok(());
        }
        // Straddles a region's edge: rare, so take the per-pixel path
        self.draw_iter(area.points().zip(colors).map(|(point, color)| Pixel(point, color)))
    }

    fn fill_solid(&mut self, area: &Rectangle, color: Self::Color) -> Result<(), Self::Error> {
        for region in self.regions.iter_mut() {
            let inside = area.intersection(&region.area);
            if !inside.is_zero_sized() {
                region.fill(&inside, color);
            }
        }
        fill_outside(self.lcd, *area, self.regions, color)
    }
}

/// Fill the parts of `area` that lie outside every region on the panel.
fn fill_outside(lcd: &mut ILI9341, area: Rectangle, regions: &[Region], color: Rgb565) -> Result<(), Infallible> {
    let Some((first, rest)) = regions.split_first() else {
        return lcd.fill_solid(&area, color);
    };
    let inside = area.intersection(&first.area);
    if inside.is_zero_sized() {
        return fill_outside(lcd, area, rest, color);
    }
    for band in around(&area, &inside) {
        fill_outside(lcd, band, rest, color)?;
    }
    Ok(())
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

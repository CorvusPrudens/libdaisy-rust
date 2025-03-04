use embedded_hal::digital::v2::InputPin;

/// A quadrature encoder driver.
///
/// This struct reads the quadrature input via polling.
pub struct Encoder<A, B> {
    a: A,
    b: B,
    decoder: QuadratureDecoder,
}

impl<A, B> Encoder<A, B>
where
    A: InputPin,
    B: InputPin,
    <A as InputPin>::Error: core::fmt::Debug,
    <B as InputPin>::Error: core::fmt::Debug,
{
    pub const fn new(a: A, b: B) -> Self {
        Self {
            a,
            b,
            decoder: QuadratureDecoder::new(),
        }
    }

    /// Update the encoder.
    ///
    /// `milliseconds` should be a monotonically increasing count.
    /// This allows the debouncing implementation to limit updates
    /// to the desired rate.
    pub fn update(&mut self, milliseconds: u32) {
        self.decoder.update(
            self.a.is_high().unwrap(),
            self.b.is_high().unwrap(),
            milliseconds,
        );
    }

    /// Get the magnitude of the last encoder turn, if any.
    pub fn increment(&self) -> Option<i32> {
        self.decoder.increment()
    }
}

#[derive(Debug, Clone)]
pub struct QuadratureDecoder {
    last_update: u32,
    a: u8,
    b: u8,
    increment: i32,
}

impl Default for QuadratureDecoder {
    fn default() -> Self {
        Self::new()
    }
}

/// A quadrature encoder decoder.
///
/// This is useful when your quadrature encoder data
/// lines are not connected directly to Daisy pins.
impl QuadratureDecoder {
    pub const fn new() -> Self {
        Self {
            last_update: 0,
            a: 0,
            b: 0,
            increment: 0,
        }
    }

    /// Update the decoder with the current state of `a` and `b`.
    ///
    /// `milliseconds` should be a monotonically increasing count.
    /// This allows the debouncing implementation to limit updates
    /// to the desired rate.
    pub fn update(&mut self, a: bool, b: bool, milliseconds: u32) {
        self.increment = 0;

        if milliseconds.saturating_sub(self.last_update) >= 1 {
            self.last_update = milliseconds;

            self.a = (self.a << 1) | a as u8;
            self.b = (self.b << 1) | b as u8;

            if (self.a & 0x03) == 0x02 && (self.b & 0x03) == 0x00 {
                self.increment = 1;
            } else if (self.b & 0x03) == 0x02 && (self.a & 0x03) == 0x00 {
                self.increment = -1;
            }
        }
    }

    /// Get the magnitude of the last encoder turn, if any.
    pub fn increment(&self) -> Option<i32> {
        match self.increment {
            0 => None,
            i => Some(i),
        }
    }
}

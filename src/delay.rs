//! Simple, cycle-based delay.

use cortex_m::asm::delay as delay_cycles;
pub use embedded_hal::blocking::delay::{DelayMs, DelayUs};

/// A type that uses CPU cycles as a delay source.
///
/// [CycleDelay] is guaranteed to block for
/// at least as many cycles as requested, but it
/// makes no claims about accuracy. Servicing
/// interrupts can make the delay extend much longer.
///
/// For accurate timings, use a hardware timer.
///
/// Delay methods are provided via `embedded-hal`'s [DelayMs]
/// and [DelayUs] traits.
///
/// ```
/// use libdaisy::delay::{CycleDelay, DelayMs};
///
/// let mut delay = CycleDelay::new();
/// // You may need to specify the literal's
/// // type to help Rust resolve the trait.
/// delay.delay_ms(10u8);
/// ```
#[derive(Default)]
pub struct CycleDelay;

impl CycleDelay {
    /// Construct a new [CycleDelay].
    pub fn new() -> Self {
        CycleDelay
    }

    /// Block for `cycles` processor cycles.
    pub fn delay_cycles(&mut self, cycles: u32) {
        delay_cycles(cycles);
    }

    /// Block for approximately `cycles` nanoseconds.
    ///
    /// This is a rough approximation with a resolution
    /// of about four nanoseconds.
    pub fn delay_ns(&mut self, ns: u32) {
        delay_cycles(ns / 4);
    }
}

macro_rules! impl_delay {
    ($ty:path) => {
        impl DelayMs<$ty> for CycleDelay {
            fn delay_ms(&mut self, ms: $ty) {
                self.delay_cycles((ms as u32).saturating_mul(crate::MILICYCLES));
            }
        }

        impl DelayUs<$ty> for CycleDelay {
            fn delay_us(&mut self, us: $ty) {
                self.delay_cycles((us as u32).saturating_mul(crate::MICROCYCLES));
            }
        }
    };
}

impl_delay!(u8);
impl_delay!(u16);
impl_delay!(u32);

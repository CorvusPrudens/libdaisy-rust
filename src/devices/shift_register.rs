use embedded_hal::digital::v2::{InputPin, OutputPin};

/// A trait for reading and updating shift registers.
///
/// This is implemented for [`ShiftRegister4021`] for up to
/// 8 parallel connections.
pub trait ShiftRegister {
    /// Update the shift register object, reading each bit sequentially
    /// in a blocking fashion.
    fn update(&mut self);

    /// Read `input` from the given `parallel` chain.
    ///
    /// # Panics
    ///
    /// Panics if `inptut` or `parallel` are out of range.
    fn read_parallel(&self, input: usize, parallel: usize) -> bool;

    /// Read `input` from the first parallel group.
    ///
    /// # Panics
    ///
    /// Panics if `inptut` is out of range.
    fn read(&self, input: usize) -> bool {
        self.read_parallel(input, 0)
    }
}

/// A 4021 shift register meta-struct.
///
/// This struct permits configurations with both series and parallel
/// configurations. Note that `PARALLEL` will need to match the
/// number of data pins for the update implementation to work.
pub struct ShiftRegister4021<C, L, D, const SERIES: usize, const PARALLEL: usize> {
    clock: C,
    latch: L,
    data: D,
    delay: u32,
    bits: [[u8; SERIES]; PARALLEL],
}

impl<C, L, D, const SERIES: usize, const PARALLEL: usize>
    ShiftRegister4021<C, L, D, SERIES, PARALLEL>
{
    /// Construct a new [`ShiftRegister4021`].
    ///
    /// `clock` and `latch` are expected to take single pins,
    /// while `data` expects a tuple of up to 8 pins.
    pub fn new(clock: C, latch: L, data: D) -> Self {
        Self {
            clock,
            latch,
            data,
            delay: 1000,
            bits: [[0u8; SERIES]; PARALLEL],
        }
    }

    /// Set the delay time between clock edges in nanoseconds.
    pub fn set_delay(&mut self, delay_ns: u32) {
        self.delay = delay_ns;
    }

    fn parallel(&self) -> usize {
        PARALLEL
    }
}

macro_rules! impl_sr {
    ($count:literal, $($ty:ident),*) => {
        impl<T, U, $($ty,)* const SERIES: usize> ShiftRegister for ShiftRegister4021<T, U, ($($ty,)*), SERIES, $count>
        where
            T: OutputPin,
            U: OutputPin,
            <T as OutputPin>::Error: core::fmt::Debug,
            <U as OutputPin>::Error: core::fmt::Debug,
            $($ty: InputPin,)*
            $(<$ty as InputPin>::Error: core::fmt::Debug,)*
        {
            #[allow(non_snake_case)]
            fn update(&mut self) {
                let mut delay = crate::delay::CycleDelay::new();

                self.clock.set_low().unwrap();
                self.latch.set_high().unwrap();

                delay.delay_ns(self.delay);

                self.latch.set_low().unwrap();

                let total = self.bits[0].len();
                let ($($ty,)*) = &self.data;
                for series in 0..total {
                    for bit in 0..8 {
                        self.clock.set_low().unwrap();
                        delay.delay_ns(self.delay);

                        let new_data: [u8; $count] = [
                            $($ty.is_high().unwrap() as u8),*
                        ];

                        for (parallel, data) in new_data.into_iter().enumerate() {
                            self.bits[parallel][series] &= !(1 << bit);
                            self.bits[parallel][series] |= data << bit;
                        }

                        self.clock.set_high().unwrap();
                        delay.delay_ns(self.delay);
                    }
                }
            }

            fn read_parallel(&self, bit: usize, parallel: usize) -> bool {
                let byte_address = bit / 8;
                let bit_address = bit % 8;

                ((self.bits[parallel][byte_address] >> bit_address) & 1) > 0
            }
        }
    };
}

impl_sr!(1, A);
impl_sr!(2, A, B);
impl_sr!(3, A, B, C);
impl_sr!(4, A, B, C, D);
impl_sr!(5, A, B, C, D, E);
impl_sr!(6, A, B, C, D, E, F);
impl_sr!(7, A, B, C, D, E, F, G);
impl_sr!(8, A, B, C, D, E, F, G, H);

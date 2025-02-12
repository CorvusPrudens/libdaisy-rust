use embedded_hal::digital::v2::OutputPin;

pub struct Multiplexer<A, D, const LENGTH: usize> {
    analog_pin: A,
    digital_pins: D,
    channels: [u32; LENGTH],
    index: u8,
}

impl<A, D, const LENGTH: usize> Multiplexer<A, D, LENGTH> {
    pub fn analog_pin(&mut self) -> &mut A {
        &mut self.analog_pin
    }

    pub fn channels(&self) -> &[u32] {
        &self.channels
    }

    pub fn update(&mut self, value: u32) {
        self.channels[self.index as usize] = value;
    }
}

macro_rules! impl_multi {
    ($count:literal, $new:ident, $($ty:ident),*) => {
        impl<A, $($ty),*> Multiplexer<A, ($($ty,)*), $count>
        where
            $($ty: OutputPin),*
        {
            /// Construct a new [Multiplexer].
            ///
            /// `digital_pins` must be a tuple of one to three output pins.
            pub fn $new(analog_pin: A, digital_pins: ($($ty,)*)) -> Self {
                Self {
                    analog_pin,
                    digital_pins,
                    channels: [const { 0 }; $count],
                    index: 0,
                }
            }

            /// Tick the multiplexer pins by one.
            #[allow(unused_assignments, non_snake_case)]
            pub fn tick(&mut self) {
                self.index = (self.index + 1) % $count;
                let ($($ty,)*) = &mut self.digital_pins;
                let mut mask = self.index;

                $(
                    let _ = $ty.set_state(((mask & 1) > 0).into());
                    mask >>= 1;
                )*
            }
        }
    };
}

impl_multi!(2, new_1, D0);
impl_multi!(4, new_2, D0, D1);
impl_multi!(8, new_3, D0, D1, D2);

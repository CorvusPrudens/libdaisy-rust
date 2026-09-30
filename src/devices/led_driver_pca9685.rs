#![allow(dead_code)]

use cortex_m::peripheral::DWT;
use embedded_hal::blocking::delay::DelayMs;
use embedded_hal::blocking::i2c::Write;

#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct LedRegister {
    pub on: u16,
    pub off: u16,
}

#[repr(C, packed)]
#[derive(Clone, Copy, Debug)]
pub struct Pca9685TransmitBuffer {
    pub register_addr: u8,
    pub leds: [LedRegister; 16],
}

impl Pca9685TransmitBuffer {
    pub const SIZE: usize = 1 + 16 * 4;

    pub const fn new() -> Self {
        Self {
            register_addr: PCA9685_LED0,
            leds: [LedRegister { on: 0, off: 0 }; 16],
        }
    }
}

impl Default for Pca9685TransmitBuffer {
    fn default() -> Self {
        Self {
            register_addr: PCA9685_LED0,
            leds: [LedRegister { on: 0, off: 0 }; 16],
        }
    }
}

pub type DmaBuffer<const N: usize> = [Pca9685TransmitBuffer; N];

const RETRY_MS: u32 = 1000;

pub struct LedDriverPca9685<I, const NUM_DRIVERS: usize> {
    i2c: I,
    draw_buffer: Option<&'static mut [u8]>,
    transmit_buffer: Option<&'static mut [u8]>,
    addresses: [u8; NUM_DRIVERS],
    /// Whether each driver has been initialized and has kept responding.
    ready: [bool; NUM_DRIVERS],
    /// DWT cycle count after which unready drivers are tried again.
    retry_at: u32,
}

// Constants
const PCA9685_I2C_BASE_ADDRESS: u8 = 0b01000000;
const PCA9685_MODE1: u8 = 0x00;
const PCA9685_MODE2: u8 = 0x01;
const PRE_SCALE_MODE: u8 = 0xFE;
const PCA9685_LED0: u8 = 0x06;

impl<I, const NUM_DRIVERS: usize> LedDriverPca9685<I, NUM_DRIVERS>
where
    I: Write,
{
    /// `i2c` must give up on a dead bus (see [`crate::i2c::I2cTimeout`]); the
    /// HAL's own blocking I2C spins forever when no driver is connected.
    pub fn new(
        mut i2c: I,
        addresses: [u8; NUM_DRIVERS],
        dma_buffer_a: &'static mut DmaBuffer<NUM_DRIVERS>,
        dma_buffer_b: &'static mut DmaBuffer<NUM_DRIVERS>,
    ) -> Self {
        let mut ready = [false; NUM_DRIVERS];
        for (ready, &addr) in ready.iter_mut().zip(&addresses) {
            *ready = Self::init_driver(&mut i2c, addr, 20).is_ok();
        }

        let len = NUM_DRIVERS * Pca9685TransmitBuffer::SIZE;

        // Initialize buffers
        for buf in dma_buffer_a.iter_mut() {
            buf.register_addr = PCA9685_LED0;
        }
        for buf in dma_buffer_b.iter_mut() {
            buf.register_addr = PCA9685_LED0;
        }

        let buffer_a_u8 =
            unsafe { core::slice::from_raw_parts_mut(dma_buffer_a.as_mut_ptr() as *mut u8, len) };
        let buffer_b_u8 =
            unsafe { core::slice::from_raw_parts_mut(dma_buffer_b.as_mut_ptr() as *mut u8, len) };

        Self {
            i2c,
            draw_buffer: Some(buffer_b_u8),
            transmit_buffer: Some(buffer_a_u8),
            addresses,
            ready,
            retry_at: DWT::cycle_count().wrapping_add(RETRY_MS * crate::MILICYCLES),
        }
    }

    /// Wake one driver and set it up, stopping at the first write it doesn't
    /// acknowledge.
    fn init_driver(i2c: &mut I, addr: u8, settle_ms: u8) -> Result<(), I::Error> {
        let address = PCA9685_I2C_BASE_ADDRESS | addr;
        let mut delay = crate::delay::CycleDelay::new();
        i2c.write(address, &[PCA9685_MODE1, 0x00])?;
        delay.delay_ms(settle_ms);
        i2c.write(address, &[PCA9685_MODE1, 0x00])?;
        delay.delay_ms(settle_ms);
        i2c.write(address, &[PCA9685_MODE1, 0b00100000])?; // Auto increment
        delay.delay_ms(settle_ms);
        // There are a few configurations in this register we may want to expose later.
        i2c.write(address, &[PCA9685_MODE2, 0b00000110]) // OE hi-z, odrv=1
    }

    pub fn set_led(&mut self, led_index: usize, brightness: f32) {
        let driver_idx = led_index / 16;
        let led_idx_in_driver = led_index % 16;

        if driver_idx >= NUM_DRIVERS {
            return;
        }

        if let Some(buffer) = &mut self.draw_buffer {
            let offset = driver_idx * Pca9685TransmitBuffer::SIZE + 1 + led_idx_in_driver * 4;

            let val = (brightness * 4095.0) as u16;
            let on = 0;
            let off = if val > 4095 { 4095 } else { val };

            buffer[offset] = (on & 0xFF) as u8;
            buffer[offset + 1] = (on >> 8) as u8;
            buffer[offset + 2] = (off & 0xFF) as u8;
            buffer[offset + 3] = (off >> 8) as u8;
        }
    }

    pub fn swap_buffers_and_transmit(&mut self) {
        // Swap buffers
        let draw = self.draw_buffer.take().unwrap();
        let transmit = self.transmit_buffer.take().unwrap();
        self.draw_buffer = Some(transmit);
        self.transmit_buffer = Some(draw);

        // Start transmission
        self.transmit();
    }

    fn transmit(&mut self) {
        let now = DWT::cycle_count();
        // Signed distance, so the counter's wrap is harmless.
        let retry = (now.wrapping_sub(self.retry_at) as i32) >= 0;
        if retry {
            self.retry_at = now.wrapping_add(RETRY_MS * crate::MILICYCLES);
        }

        for driver_idx in 0..NUM_DRIVERS {
            if !self.ready[driver_idx] {
                // The oscillator needs 500us after waking; this runs in the
                // main loop, so no longer than that.
                if !retry
                    || Self::init_driver(&mut self.i2c, self.addresses[driver_idx], 1).is_err()
                {
                    continue;
                }
                self.ready[driver_idx] = true;
            }

            let address = PCA9685_I2C_BASE_ADDRESS | self.addresses[driver_idx];

            let buffer = self.transmit_buffer.as_mut().unwrap();
            let start = driver_idx * Pca9685TransmitBuffer::SIZE;
            let end = start + Pca9685TransmitBuffer::SIZE;
            let bytes = &buffer[start..end];
            if self.i2c.write(address, bytes).is_err() {
                self.ready[driver_idx] = false;
            }
        }
    }

    pub fn poll(&mut self) {
        // No-op for blocking
    }
}

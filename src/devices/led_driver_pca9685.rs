#![allow(dead_code)]

use crate::hal;
use hal::i2c::I2c;
use hal::prelude::*;
use stm32h7xx_hal::prelude::_embedded_hal_blocking_i2c_Write as Write;

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

pub struct LedDriverPca9685<I2CN, const NUM_DRIVERS: usize> {
    i2c: I2c<I2CN>,
    draw_buffer: Option<&'static mut [u8]>,
    transmit_buffer: Option<&'static mut [u8]>,
    addresses: [u8; NUM_DRIVERS],
}

// Constants
const PCA9685_I2C_BASE_ADDRESS: u8 = 0b01000000;
const PCA9685_MODE1: u8 = 0x00;
const PCA9685_MODE2: u8 = 0x01;
const PRE_SCALE_MODE: u8 = 0xFE;
const PCA9685_LED0: u8 = 0x06;

impl<I2CN, const NUM_DRIVERS: usize> LedDriverPca9685<I2CN, NUM_DRIVERS>
where
    I2c<I2CN>: Write,
{
    pub fn new(
        mut i2c: I2c<I2CN>,
        addresses: [u8; NUM_DRIVERS],
        dma_buffer_a: &'static mut DmaBuffer<NUM_DRIVERS>,
        dma_buffer_b: &'static mut DmaBuffer<NUM_DRIVERS>,
    ) -> Self {
        // Init drivers
        for &addr in &addresses {
            let address = PCA9685_I2C_BASE_ADDRESS | addr;
            let _ = i2c.write(address, &[PCA9685_MODE1, 0x00]);
            crate::delay::CycleDelay::new().delay_ms(20u8);
            let _ = i2c.write(address, &[PCA9685_MODE1, 0x00]);
            crate::delay::CycleDelay::new().delay_ms(20u8);
            let _ = i2c.write(address, &[PCA9685_MODE1, 0b00100000]); // Auto increment
            crate::delay::CycleDelay::new().delay_ms(20u8);
            // There are a few configurations in this register we may want to expose later.
            let _ = i2c.write(address, &[PCA9685_MODE2, 0b00000110]); // OE hi-z, odrv=1
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
        }
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
        for driver_idx in 0..NUM_DRIVERS {
            let address = PCA9685_I2C_BASE_ADDRESS | self.addresses[driver_idx];

            let buffer = self.transmit_buffer.as_mut().unwrap();
            let start = driver_idx * Pca9685TransmitBuffer::SIZE;
            let end = start + Pca9685TransmitBuffer::SIZE;
            let bytes = unsafe {
                core::mem::transmute::<&mut [u8], &'static mut [u8]>(&mut buffer[start..end])
            };

            let _ = self.i2c.write(address, bytes);
        }
    }

    pub fn poll(&mut self) {
        // No-op for blocking
    }
}

const GAMMA_TABLE: [u16; 256] = [
    0, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2, 2, 2, 2, 2, 2, 3, 3, 4, 4, 5, 5, 6, 7, 8, 8, 9, 10,
    11, 12, 13, 15, 16, 17, 18, 20, 21, 23, 25, 26, 28, 30, 32, 34, 36, 38, 40, 43, 45, 48, 50, 53,
    56, 59, 62, 65, 68, 71, 75, 78, 82, 85, 89, 93, 97, 101, 105, 110, 114, 119, 123, 128, 133,
    138, 143, 149, 154, 159, 165, 171, 177, 183, 189, 195, 202, 208, 215, 222, 229, 236, 243, 250,
    258, 266, 273, 281, 290, 298, 306, 315, 324, 332, 341, 351, 360, 369, 379, 389, 399, 409, 419,
    430, 440, 451, 462, 473, 485, 496, 508, 520, 532, 544, 556, 569, 582, 594, 608, 621, 634, 648,
    662, 676, 690, 704, 719, 734, 749, 764, 779, 795, 811, 827, 843, 859, 876, 893, 910, 927, 944,
    962, 980, 998, 1016, 1034, 1053, 1072, 1091, 1110, 1130, 1150, 1170, 1190, 1210, 1231, 1252,
    1273, 1294, 1316, 1338, 1360, 1382, 1404, 1427, 1450, 1473, 1497, 1520, 1544, 1568, 1593, 1617,
    1642, 1667, 1693, 1718, 1744, 1770, 1797, 1823, 1850, 1877, 1905, 1932, 1960, 1988, 2017, 2045,
    2074, 2103, 2133, 2162, 2192, 2223, 2253, 2284, 2315, 2346, 2378, 2410, 2442, 2474, 2507, 2540,
    2573, 2606, 2640, 2674, 2708, 2743, 2778, 2813, 2849, 2884, 2920, 2957, 2993, 3030, 3067, 3105,
    3143, 3181, 3219, 3258, 3297, 3336, 3376, 3416, 3456, 3496, 3537, 3578, 3619, 3661, 3703, 3745,
    3788, 3831, 3874, 3918, 3962, 4006, 4050, 4095,
];

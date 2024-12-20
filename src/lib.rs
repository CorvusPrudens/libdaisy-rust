#![no_std]
#![allow(dead_code)]

use stm32h7xx_hal::time::Hertz;

pub const MILLI: u32 = 1_000;
pub const MICRO: u32 = 1_000_000;
pub const AUDIO_FRAME_RATE_HZ: u32 = 1_000;
pub const AUDIO_BLOCK_SIZE: u16 = 48;
pub const AUDIO_SAMPLE_RATE: usize = 48_000;
pub const AUDIO_SAMPLE_HZ: Hertz = Hertz::from_raw(48_000);
pub const CLOCK_RATE_HZ: Hertz = Hertz::from_raw(480_000_000_u32);

pub const MILICYCLES: u32 = CLOCK_RATE_HZ.raw() / MILLI;
pub const MICROCYCLES: u32 = CLOCK_RATE_HZ.raw() / MICRO;

pub type FrameTimer = stm32h7xx_hal::timer::Timer<stm32h7xx_hal::stm32::TIM2>;

pub use stm32h7xx_hal as hal;

pub mod audio;
pub mod bootloader;
pub mod delay;
pub mod flash;
pub mod gpio;
pub mod hid;
pub mod logger;
pub mod mpu;
pub mod prelude;
pub mod sdmmc;
pub mod sdram;
pub mod system;

//! Contains setup for Daisy board hardware.
#![allow(dead_code)]

use hal::rcc::CoreClocks;
use log::info;
use stm32h7xx_hal::{
    gpio,
    prelude::*,
    rcc, stm32,
    time::{Hertz, MegaHertz},
};

use crate::*;

pub mod patch_sm;
pub mod seed;

pub use patch_sm::{MinimalPatchSmSystem, PatchSmSystem};
pub use seed::{MinimalSeedSystem, SeedSystem};

pub enum Board {
    Seed(seed::Version),
    PatchSm,
}

const START_OF_DRAM2: u32 = 0x30000000;
const DMA_MEM_SIZE: usize = 32 * 1024;

const HSE_CLOCK_MHZ: MegaHertz = MegaHertz::from_raw(16);
const HCLK_MHZ: MegaHertz = MegaHertz::from_raw(200);
const HCLK2_MHZ: MegaHertz = MegaHertz::from_raw(200);

// PCLKx
const PCLK_HZ: Hertz = Hertz::from_raw(CLOCK_RATE_HZ.raw() / 4);
// 49_152_344
// PLL1
const PLL1_P_HZ: Hertz = CLOCK_RATE_HZ;
const PLL1_Q_HZ: Hertz = Hertz::from_raw(CLOCK_RATE_HZ.raw() / 18);
const PLL1_R_HZ: Hertz = Hertz::from_raw(CLOCK_RATE_HZ.raw() / 32);
// PLL2
const PLL2_P_HZ: Hertz = Hertz::from_raw(4_000_000);
const PLL2_Q_HZ: Hertz = Hertz::from_raw(PLL2_P_HZ.raw() / 2); // No divder given, what's the default?
const PLL2_R_HZ: Hertz = Hertz::from_raw(PLL2_P_HZ.raw() / 4); // No divder given, what's the default?

fn init_clocks(
    mut pwr: stm32::PWR,
    mut rcc: stm32::RCC,
    syscfg: &stm32::SYSCFG,
    audio_sample_rate: Hertz,
) -> rcc::Ccdr {
    // Power
    initialize_backup_sram(&mut pwr, &mut rcc);
    let pwr = pwr.constrain();
    let vos = pwr.vos0(syscfg).freeze();

    let PLL3_P_HZ = Hertz::from_raw(audio_sample_rate.raw() * 257);
    let PLL3_Q_HZ = Hertz::from_raw(PLL3_P_HZ.raw());
    let PLL3_R_HZ = Hertz::from_raw(PLL3_P_HZ.raw());

    rcc.constrain()
        .use_hse(HSE_CLOCK_MHZ.convert())
        .sys_ck(CLOCK_RATE_HZ)
        .pclk1(PCLK_HZ) // DMA clock
        // PLL1
        .pll1_strategy(rcc::PllConfigStrategy::Iterative)
        .pll1_p_ck(PLL1_P_HZ)
        .pll1_q_ck(PLL1_Q_HZ)
        .pll1_r_ck(PLL1_R_HZ)
        // PLL2
        .pll2_p_ck(PLL2_P_HZ) // Default adc_ker_ck_input
        // .pll2_q_ck(PLL2_Q_HZ)
        // .pll2_r_ck(PLL2_R_HZ)
        // PLL3
        .pll3_strategy(rcc::PllConfigStrategy::Fractional)
        .pll3_p_ck(PLL3_P_HZ) // used for SAI1
        .pll3_q_ck(PLL3_Q_HZ)
        .pll3_r_ck(PLL3_R_HZ)
        .freeze(vos, syscfg)
}

pub fn initialize_backup_sram(pwr: &mut stm32::PWR, rcc: &mut stm32::RCC) {
    pwr.cr1.modify(|_, w| w.dbp().set_bit());
    pwr.cr2.modify(|_, w| w.bren().set_bit());

    loop {
        if pwr.cr1.read().dbp().bit_is_set() {
            break;
        }
    }

    rcc.ahb4enr.modify(|_, w| w.bkpramen().set_bit());
    // read it back
    let _bit = rcc.ahb4enr.read().bkpramen().bit();
}

pub struct InternalUsbPins {
    pub dp: gpio::gpioa::PA12<gpio::Analog>,
    pub dm: gpio::gpioa::PA11<gpio::Analog>,
}

/// All peripherals and other resources required for the system
pub struct SystemResources<'a> {
    pub clocks: &'a CoreClocks,
    pub adc1: stm32::ADC1,
    pub adc2: stm32::ADC2,
    pub adc12_rec: rcc::rec::Adc12,
    pub dac: stm32::DAC,
    pub dac12_rec: rcc::rec::Dac12,
    pub syst: stm32::SYST,
    pub mpu: &'a mut stm32::MPU,
    pub scb: &'a mut stm32::SCB,
    pub dcb: &'a mut stm32::DCB,
    pub dwt: &'a mut stm32::DWT,
    pub fmc: stm32::FMC,
    pub fmc_rec: rcc::rec::Fmc,
    pub i2c2: stm32::I2C2,
    pub i2c2_rec: rcc::rec::I2c2,
    pub cpuid: &'a mut cortex_m::peripheral::CPUID,
    pub qspi: stm32::QUADSPI,
    pub qspi_rec: rcc::rec::Qspi,

    pub sai1: stm32::SAI1,
    pub sai1_rec: rcc::rec::Sai1,

    pub gpioa: stm32::GPIOA,
    pub gpioa_rec: rcc::rec::Gpioa,

    pub gpiob: stm32::GPIOB,
    pub gpiob_rec: rcc::rec::Gpiob,

    pub gpioc: stm32::GPIOC,
    pub gpioc_rec: rcc::rec::Gpioc,

    pub gpiod: stm32::GPIOD,
    pub gpiod_rec: rcc::rec::Gpiod,

    pub gpioe: stm32::GPIOE,
    pub gpioe_rec: rcc::rec::Gpioe,

    pub gpiof: stm32::GPIOF,
    pub gpiof_rec: rcc::rec::Gpiof,

    pub gpiog: stm32::GPIOG,
    pub gpiog_rec: rcc::rec::Gpiog,

    pub gpioh: stm32::GPIOH,
    pub gpioh_rec: rcc::rec::Gpioh,

    pub gpioi: stm32::GPIOI,
    pub gpioi_rec: rcc::rec::Gpioi,

    pub dma1: stm32::DMA1,
    pub dma1_rec: rcc::rec::Dma1,

    pub block_size: usize,
}

fn log_clocks(ccdr: &stm32h7xx_hal::rcc::Ccdr) {
    info!("Core {}", ccdr.clocks.c_ck());
    info!("hclk {}", ccdr.clocks.hclk());
    info!("pclk1 {}", ccdr.clocks.pclk1());
    info!("pclk2 {}", ccdr.clocks.pclk2());
    info!("pclk3 {}", ccdr.clocks.pclk2());
    info!("pclk4 {}", ccdr.clocks.pclk4());
    info!(
        "PLL1\nP: {:?}\nQ: {:?}\nR: {:?}",
        ccdr.clocks.pll1_p_ck(),
        ccdr.clocks.pll1_q_ck(),
        ccdr.clocks.pll1_r_ck()
    );
    info!(
        "PLL2\nP: {:?}\nQ: {:?}\nR: {:?}",
        ccdr.clocks.pll2_p_ck(),
        ccdr.clocks.pll2_q_ck(),
        ccdr.clocks.pll2_r_ck()
    );
    info!(
        "PLL3\nP: {:?}\nQ: {:?}\nR: {:?}",
        ccdr.clocks.pll3_p_ck(),
        ccdr.clocks.pll3_q_ck(),
        ccdr.clocks.pll3_r_ck()
    );
}

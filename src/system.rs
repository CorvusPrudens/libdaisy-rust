//! Contains setup for Daisy board hardware.
#![allow(dead_code)]

use hal::rcc::CoreClocks;
use log::info;
use stm32h7xx_hal::{
    adc,
    delay::Delay,
    dma, gpio,
    prelude::*,
    rcc, stm32,
    time::{Hertz, MegaHertz},
};

use crate::{audio::Audio, *};

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

const PLL3_P_HZ: Hertz = Hertz::from_raw(AUDIO_SAMPLE_HZ.raw() * 257);
const PLL3_Q_HZ: Hertz = Hertz::from_raw(PLL3_P_HZ.raw());
const PLL3_R_HZ: Hertz = Hertz::from_raw(PLL3_P_HZ.raw());

pub struct System {
    pub gpio: crate::gpio::GPIO,
    pub audio: audio::Audio,
    pub adc1: adc::Adc<stm32::ADC1, adc::Disabled>,
    pub adc2: adc::Adc<stm32::ADC2, adc::Disabled>,
    pub sdram: &'static mut [f32],
    pub flash: crate::flash::Flash,
    pub internal_usb: Option<InternalUsbPins>,
    pub delay: Delay,
}

pub struct MinimalSystem {
    pub gpio: crate::gpio::GPIO,
    pub flash: crate::flash::Flash,
    pub internal_usb: Option<InternalUsbPins>,
    pub delay: Delay,
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

impl MinimalSystem {
    /// Initialize clocks
    pub fn init_clocks(
        mut pwr: stm32::PWR,
        mut rcc: stm32::RCC,
        syscfg: &stm32::SYSCFG,
    ) -> rcc::Ccdr {
        // Power
        initialize_backup_sram(&mut pwr, &mut rcc);
        let pwr = pwr.constrain();
        let vos = pwr.vos0(syscfg).freeze();

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

    pub fn new(resources: SystemResources) -> Self {
        let mut delay = Delay::new(resources.syst, *resources.clocks);

        let gpioa = resources.gpioa.split(resources.gpioa_rec);
        let gpiob = resources.gpiob.split(resources.gpiob_rec);
        let gpioc = resources.gpioc.split(resources.gpioc_rec);
        let gpiod = resources.gpiod.split(resources.gpiod_rec);
        let gpioe = resources.gpioe.split(resources.gpioe_rec);
        let gpiof = resources.gpiof.split(resources.gpiof_rec);
        let gpiog = resources.gpiog.split(resources.gpiog_rec);
        let gpioh = resources.gpioh.split(resources.gpioh_rec);
        let gpioi = resources.gpioi.split(resources.gpioi_rec);

        // Set up GPIOs
        let gpio = crate::gpio::GPIO::init(
            gpioc.pc7,
            gpiog.pg3,
            Some(gpiob.pb12),
            Some(gpioc.pc11),
            Some(gpioc.pc10),
            Some(gpioc.pc9),
            Some(gpioc.pc8),
            Some(gpiod.pd2),
            Some(gpioc.pc12),
            Some(gpiog.pg10),
            Some(gpiog.pg11),
            Some(gpiob.pb4),
            Some(gpiob.pb5),
            Some(gpiob.pb8),
            Some(gpiob.pb9),
            Some(gpiob.pb6),
            Some(gpiob.pb7),
            Some(gpioc.pc0),
            Some(gpioa.pa3),
            Some(gpiob.pb1),
            Some(gpioa.pa7),
            Some(gpioa.pa6),
            Some(gpioc.pc1),
            Some(gpioc.pc4),
            Some(gpioa.pa5),
            Some(gpioa.pa4),
            Some(gpioa.pa1),
            Some(gpioa.pa0),
            Some(gpiod.pd11),
            Some(gpiog.pg9),
            Some(gpioa.pa2),
            Some(gpiob.pb14),
            Some(gpiob.pb15),
            None,
            None,
        );

        System::init_debug(resources.dcb, resources.dwt);

        // set up flash
        let flash = crate::flash::Flash::new(
            resources.qspi,
            resources.qspi_rec,
            resources.clocks,
            gpiof.pf6,
            gpiof.pf7,
            gpiof.pf8,
            gpiof.pf9,
            gpiof.pf10,
            gpiog.pg6,
        );

        info!("Minimal system initialization complete.");

        // grab internal usb pins
        let internal_usb = InternalUsbPins {
            dp: gpioa.pa12,
            dm: gpioa.pa11,
        };

        Self {
            gpio,
            flash,
            internal_usb: Some(internal_usb),
            delay,
        }
    }
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

#[macro_export]
macro_rules! system_init {
    ($core:ident, $device:ident, $ccdr:ident) => {
        libdaisy::system_init!($core, $device, $ccdr, libdaisy::audio::BLOCK_SIZE_MAX);
    };
    ($core:ident, $device:ident, $ccdr:ident, $block_size:expr) => {{
        let resources = libdaisy::system::SystemResources {
            clocks: &$ccdr.clocks,
            adc1: $device.ADC1,
            adc2: $device.ADC2,
            adc12_rec: $ccdr.peripheral.ADC12,
            syst: $core.SYST,
            mpu: &mut $core.MPU,
            scb: &mut $core.SCB,
            dcb: &mut $core.DCB,
            dwt: &mut $core.DWT,
            fmc: $device.FMC,
            fmc_rec: $ccdr.peripheral.FMC,
            i2c2: $device.I2C2,
            i2c2_rec: $ccdr.peripheral.I2C2,
            cpuid: &mut $core.CPUID,
            qspi: $device.QUADSPI,
            qspi_rec: $ccdr.peripheral.QSPI,
            sai1: $device.SAI1,
            sai1_rec: $ccdr.peripheral.SAI1,
            gpioa: $device.GPIOA,
            gpioa_rec: $ccdr.peripheral.GPIOA,
            gpiob: $device.GPIOB,
            gpiob_rec: $ccdr.peripheral.GPIOB,
            gpioc: $device.GPIOC,
            gpioc_rec: $ccdr.peripheral.GPIOC,
            gpiod: $device.GPIOD,
            gpiod_rec: $ccdr.peripheral.GPIOD,
            gpioe: $device.GPIOE,
            gpioe_rec: $ccdr.peripheral.GPIOE,
            gpiof: $device.GPIOF,
            gpiof_rec: $ccdr.peripheral.GPIOF,
            gpiog: $device.GPIOG,
            gpiog_rec: $ccdr.peripheral.GPIOG,
            gpioh: $device.GPIOH,
            gpioh_rec: $ccdr.peripheral.GPIOH,
            gpioi: $device.GPIOI,
            gpioi_rec: $ccdr.peripheral.GPIOI,
            dma1: $device.DMA1,
            dma1_rec: $ccdr.peripheral.DMA1,
            block_size: $block_size,
        };

        libdaisy::system::System::init(resources)
    }};
}

#[macro_export]
macro_rules! minimal_init {
    ($core:ident, $device:ident, $ccdr:ident) => {{
        let resources = ::libdaisy::system::SystemResources {
            clocks: &$ccdr.clocks,
            adc1: $device.ADC1,
            adc2: $device.ADC2,
            adc12_rec: $ccdr.peripheral.ADC12,
            syst: $core.SYST,
            mpu: &mut $core.MPU,
            scb: &mut $core.SCB,
            dcb: &mut $core.DCB,
            dwt: &mut $core.DWT,
            fmc: $device.FMC,
            fmc_rec: $ccdr.peripheral.FMC,
            i2c2: $device.I2C2,
            i2c2_rec: $ccdr.peripheral.I2C2,
            cpuid: &mut $core.CPUID,
            qspi: $device.QUADSPI,
            qspi_rec: $ccdr.peripheral.QSPI,
            sai1: $device.SAI1,
            sai1_rec: $ccdr.peripheral.SAI1,
            gpioa: $device.GPIOA,
            gpioa_rec: $ccdr.peripheral.GPIOA,
            gpiob: $device.GPIOB,
            gpiob_rec: $ccdr.peripheral.GPIOB,
            gpioc: $device.GPIOC,
            gpioc_rec: $ccdr.peripheral.GPIOC,
            gpiod: $device.GPIOD,
            gpiod_rec: $ccdr.peripheral.GPIOD,
            gpioe: $device.GPIOE,
            gpioe_rec: $ccdr.peripheral.GPIOE,
            gpiof: $device.GPIOF,
            gpiof_rec: $ccdr.peripheral.GPIOF,
            gpiog: $device.GPIOG,
            gpiog_rec: $ccdr.peripheral.GPIOG,
            gpioh: $device.GPIOH,
            gpioh_rec: $ccdr.peripheral.GPIOH,
            gpioi: $device.GPIOI,
            gpioi_rec: $ccdr.peripheral.GPIOI,
            dma1: $device.DMA1,
            dma1_rec: $ccdr.peripheral.DMA1,
            block_size: 0,
        };

        ::libdaisy::system::MinimalSystem::new(resources)
    }};
}

#[derive(Clone, Copy)]
pub enum Version {
    Seed,
    Seed1_1,
    Seed2DFM,
}

impl System {
    fn detect_version(
        s2dfm_pin: hal::gpio::gpiod::PD4<hal::gpio::Analog>,
        seed1_1_pin: hal::gpio::gpiod::PD3<hal::gpio::Analog>,
    ) -> Version {
        let s2dfm_pin = s2dfm_pin.into_pull_up_input();
        let seed1_1_pin = seed1_1_pin.into_pull_up_input();

        let state = (seed1_1_pin.is_low(), s2dfm_pin.is_low());

        // Deinitialize the pins after reading
        s2dfm_pin.into_analog();
        seed1_1_pin.into_analog();

        match state {
            (true, _) => Version::Seed1_1,
            (false, true) => Version::Seed2DFM,
            _ => Version::Seed,
        }
    }

    /// Initialize clocks
    pub fn init_clocks(
        mut pwr: stm32::PWR,
        mut rcc: stm32::RCC,
        syscfg: &stm32::SYSCFG,
    ) -> rcc::Ccdr {
        // Power
        initialize_backup_sram(&mut pwr, &mut rcc);
        let pwr = pwr.constrain();
        let vos = pwr.vos0(syscfg).freeze();

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

    /// Set up cache
    pub fn init_cache(
        scb: &mut cortex_m::peripheral::SCB,
        cpuid: &mut cortex_m::peripheral::CPUID,
    ) {
        scb.enable_icache();
        scb.enable_dcache(cpuid);
    }

    /// Enable debug
    pub fn init_debug(dcb: &mut cortex_m::peripheral::DCB, dwt: &mut cortex_m::peripheral::DWT) {
        dcb.enable_trace();
        cortex_m::peripheral::DWT::unlock();
        dwt.enable_cycle_counter();
    }

    /// Batteries included initialization
    pub fn init(resources: SystemResources) -> System {
        info!("Starting system init");
        info!("Set up up DMA RAM in DRAM2...");
        crate::mpu::init_dma(
            resources.mpu,
            resources.scb,
            START_OF_DRAM2 as *mut u32,
            DMA_MEM_SIZE,
        );

        let mut delay = Delay::new(resources.syst, *resources.clocks);

        // Set up ADCs
        let (adc1, adc2) = adc::adc12(
            resources.adc1,
            resources.adc2,
            4.MHz(),
            &mut delay,
            resources.adc12_rec,
            resources.clocks,
        );

        Self::init_debug(resources.dcb, resources.dwt);

        info!("Setting up GPIOs...");
        let gpioa = resources.gpioa.split(resources.gpioa_rec);
        let gpiob = resources.gpiob.split(resources.gpiob_rec);
        let gpioc = resources.gpioc.split(resources.gpioc_rec);
        let gpiod = resources.gpiod.split(resources.gpiod_rec);
        let gpioe = resources.gpioe.split(resources.gpioe_rec);
        let gpiof = resources.gpiof.split(resources.gpiof_rec);
        let gpiog = resources.gpiog.split(resources.gpiog_rec);
        let gpioh = resources.gpioh.split(resources.gpioh_rec);
        let gpioi = resources.gpioi.split(resources.gpioi_rec);

        // Configure SDRAM
        info!("Setting up SDRAM...");
        let sdram = crate::sdram::Sdram::new(
            resources.fmc,
            resources.fmc_rec,
            resources.clocks,
            &mut delay,
            resources.scb,
            resources.mpu,
            gpiod.pd0,
            gpiod.pd1,
            gpiod.pd8,
            gpiod.pd9,
            gpiod.pd10,
            gpiod.pd14,
            gpiod.pd15,
            gpioe.pe0,
            gpioe.pe1,
            gpioe.pe7,
            gpioe.pe8,
            gpioe.pe9,
            gpioe.pe10,
            gpioe.pe11,
            gpioe.pe12,
            gpioe.pe13,
            gpioe.pe14,
            gpioe.pe15,
            gpiof.pf0,
            gpiof.pf1,
            gpiof.pf2,
            gpiof.pf3,
            gpiof.pf4,
            gpiof.pf5,
            gpiof.pf11,
            gpiof.pf12,
            gpiof.pf13,
            gpiof.pf14,
            gpiof.pf15,
            gpiog.pg0,
            gpiog.pg1,
            gpiog.pg2,
            gpiog.pg4,
            gpiog.pg5,
            gpiog.pg8,
            gpiog.pg15,
            gpioh.ph2,
            gpioh.ph3,
            gpioh.ph5,
            gpioh.ph8,
            gpioh.ph9,
            gpioh.ph10,
            gpioh.ph11,
            gpioh.ph12,
            gpioh.ph13,
            gpioh.ph14,
            gpioh.ph15,
            gpioi.pi0,
            gpioi.pi1,
            gpioi.pi2,
            gpioi.pi3,
            gpioi.pi4,
            gpioi.pi5,
            gpioi.pi6,
            gpioi.pi7,
            gpioi.pi9,
            gpioi.pi10,
        )
        .into();

        let dma1_streams = dma::dma::StreamsTuple::new(resources.dma1, resources.dma1_rec);

        info!("Set up Audio...");
        let version = Self::detect_version(gpiod.pd4, gpiod.pd3);

        let audio = Audio::new(
            dma1_streams.0,
            dma1_streams.1,
            resources.sai1,
            resources.sai1_rec,
            resources.i2c2,
            resources.i2c2_rec,
            gpioe.pe2,
            gpioe.pe3,
            gpioe.pe4,
            gpioe.pe5,
            gpioe.pe6,
            gpioh.ph4,
            gpiob.pb11,
            resources.clocks,
            version,
            &mut delay,
            resources.block_size,
        );

        let (d31, d32) = match version {
            Version::Seed2DFM => (Some(gpioc.pc2), Some(gpioc.pc3)),
            _ => (None, None),
        };

        // Set up GPIOs
        let gpio = crate::gpio::GPIO::init(
            gpioc.pc7,
            gpiog.pg3,
            Some(gpiob.pb12),
            Some(gpioc.pc11),
            Some(gpioc.pc10),
            Some(gpioc.pc9),
            Some(gpioc.pc8),
            Some(gpiod.pd2),
            Some(gpioc.pc12),
            Some(gpiog.pg10),
            Some(gpiog.pg11),
            Some(gpiob.pb4),
            Some(gpiob.pb5),
            Some(gpiob.pb8),
            Some(gpiob.pb9),
            Some(gpiob.pb6),
            Some(gpiob.pb7),
            Some(gpioc.pc0),
            Some(gpioa.pa3),
            Some(gpiob.pb1),
            Some(gpioa.pa7),
            Some(gpioa.pa6),
            Some(gpioc.pc1),
            Some(gpioc.pc4),
            Some(gpioa.pa5),
            Some(gpioa.pa4),
            Some(gpioa.pa1),
            Some(gpioa.pa0),
            Some(gpiod.pd11),
            Some(gpiog.pg9),
            Some(gpioa.pa2),
            Some(gpiob.pb14),
            Some(gpiob.pb15),
            d31,
            d32,
        );

        // Set up cache
        Self::init_cache(resources.scb, resources.cpuid);

        info!("System init done!");

        // set up flash
        let flash = crate::flash::Flash::new(
            resources.qspi,
            resources.qspi_rec,
            resources.clocks,
            gpiof.pf6,
            gpiof.pf7,
            gpiof.pf8,
            gpiof.pf9,
            gpiof.pf10,
            gpiog.pg6,
        );

        // grab internal usb pins
        let internal_usb = InternalUsbPins {
            dp: gpioa.pa12,
            dm: gpioa.pa11,
        };

        System {
            gpio,
            audio,
            adc1,
            adc2,
            sdram,
            flash,
            internal_usb: Some(internal_usb),
            delay,
        }
    }
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

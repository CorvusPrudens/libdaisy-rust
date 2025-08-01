use log::info;
use stm32h7xx_hal::{adc, dac, delay::Delay, dma, prelude::*, rcc, stm32};

use crate::{audio::Audio, *};

use super::*;

pub struct PatchSmSystem {
    pub gpio: crate::gpio::PatchSmGPIO,
    pub audio: audio::Audio,
    pub adc1: adc::Adc<stm32::ADC1, adc::Disabled>,
    pub adc2: adc::Adc<stm32::ADC2, adc::Disabled>,
    pub c1: Option<dac::C1<stm32::DAC, dac::Disabled>>,
    pub c2: Option<dac::C2<stm32::DAC, dac::Disabled>>,
    pub sdram: &'static mut [f32],
    pub flash: crate::flash::Flash,
    pub internal_usb: Option<InternalUsbPins>,
    pub delay: Delay,
}

pub struct MinimalPatchSmSystem {
    pub gpio: crate::gpio::PatchSmGPIO,
    pub flash: crate::flash::Flash,
    pub internal_usb: Option<InternalUsbPins>,
    pub delay: Delay,
}

impl MinimalPatchSmSystem {
    /// Initialize clocks
    pub fn init_clocks(pwr: stm32::PWR, rcc: stm32::RCC, syscfg: &stm32::SYSCFG) -> rcc::Ccdr {
        init_clocks(pwr, rcc, syscfg)
    }

    pub fn new(resources: SystemResources) -> Self {
        let delay = Delay::new(resources.syst, *resources.clocks);

        let gpioa = resources.gpioa.split(resources.gpioa_rec);
        let gpiob = resources.gpiob.split(resources.gpiob_rec);
        let gpioc = resources.gpioc.split(resources.gpioc_rec);
        let gpiod = resources.gpiod.split(resources.gpiod_rec);
        let gpiof = resources.gpiof.split(resources.gpiof_rec);
        let gpiog = resources.gpiog.split(resources.gpiog_rec);

        // Set up GPIOs
        let gpio = crate::gpio::PatchSmGPIO::init(
            gpioc.pc7,
            gpiog.pg3,
            Some(gpioa.pa1),
            Some(gpioa.pa0),
            Some(gpiob.pb14),
            Some(gpiob.pb15),
            Some(gpioc.pc14),
            Some(gpioc.pc13),
            Some(gpiob.pb8),
            Some(gpiob.pb9),
            Some(gpiog.pg14),
            Some(gpiog.pg13),
            None,
            Some(gpioa.pa7),
            Some(gpioa.pa2),
            Some(gpioa.pa6),
            Some(gpioa.pa3),
            Some(gpiob.pb1),
            Some(gpioc.pc4),
            Some(gpioc.pc0),
            Some(gpioc.pc1),
            None,
            Some(gpiob.pb4),
            Some(gpioc.pc11),
            Some(gpioc.pc10),
            Some(gpioc.pc9),
            Some(gpioc.pc8),
            Some(gpioc.pc12),
            Some(gpiod.pd2),
            Some(gpioc.pc2),
            Some(gpioc.pc3),
            Some(gpiod.pd3),
        );

        PatchSmSystem::init_debug(resources.dcb, resources.dwt);

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

#[macro_export]
macro_rules! patch_sm_system_init {
    ($core:ident, $device:ident, $ccdr:ident) => {
        libdaisy::system_init!($core, $device, $ccdr, libdaisy::audio::BLOCK_SIZE_MAX);
    };
    ($core:ident, $device:ident, $ccdr:ident, $block_size:expr) => {{
        let resources = libdaisy::system::SystemResources {
            clocks: &$ccdr.clocks,
            adc1: $device.ADC1,
            adc2: $device.ADC2,
            adc12_rec: $ccdr.peripheral.ADC12,
            dac: $device.DAC,
            dac12_rec: $ccdr.peripheral.DAC12,
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

        libdaisy::system::PatchSmSystem::init(resources)
    }};
}

#[macro_export]
macro_rules! patch_sm_minimal_init {
    ($core:ident, $device:ident, $ccdr:ident) => {{
        let resources = ::libdaisy::system::SystemResources {
            clocks: &$ccdr.clocks,
            adc1: $device.ADC1,
            adc2: $device.ADC2,
            adc12_rec: $ccdr.peripheral.ADC12,
            dac: $device.DAC,
            dac12_rec: $ccdr.peripheral.DAC12,
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

        ::libdaisy::system::MinimalPatchSmSystem::new(resources)
    }};
}

impl PatchSmSystem {
    /// Initialize clocks
    pub fn init_clocks(pwr: stm32::PWR, rcc: stm32::RCC, syscfg: &stm32::SYSCFG) -> rcc::Ccdr {
        init_clocks(pwr, rcc, syscfg)
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
    pub fn init(resources: SystemResources) -> PatchSmSystem {
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
        let audio = Audio::patch_sm(
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
            &mut delay,
            resources.block_size,
        );

        // Set up GPIOs
        let gpio = crate::gpio::PatchSmGPIO::init(
            gpioc.pc7,
            gpiog.pg3,
            Some(gpioa.pa1),
            Some(gpioa.pa0),
            Some(gpiob.pb14),
            Some(gpiob.pb15),
            Some(gpioc.pc14),
            Some(gpioc.pc13),
            Some(gpiob.pb8),
            Some(gpiob.pb9),
            Some(gpiog.pg14),
            Some(gpiog.pg13),
            None,
            Some(gpioa.pa7),
            Some(gpioa.pa2),
            Some(gpioa.pa6),
            Some(gpioa.pa3),
            Some(gpiob.pb1),
            Some(gpioc.pc4),
            Some(gpioc.pc0),
            Some(gpioc.pc1),
            None,
            Some(gpiob.pb4),
            Some(gpioc.pc11),
            Some(gpioc.pc10),
            Some(gpioc.pc9),
            Some(gpioc.pc8),
            Some(gpioc.pc12),
            Some(gpiod.pd2),
            Some(gpioc.pc2),
            Some(gpioc.pc3),
            Some(gpiod.pd3),
        );

        // Set up cache
        Self::init_cache(resources.scb, resources.cpuid);

        info!("Set up DAC...");
        let (c1, c2) = dac::dac(resources.dac, (gpioa.pa4, gpioa.pa5), resources.dac12_rec);

        info!("Patch Submodule system init done!");

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

        PatchSmSystem {
            gpio,
            audio,
            adc1,
            adc2,
            c1: Some(c1),
            c2: Some(c2),
            sdram,
            flash,
            internal_usb: Some(internal_usb),
            delay,
        }
    }
}

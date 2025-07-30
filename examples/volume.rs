//! examples/volume.rs
#![allow(unused_imports)]
#![no_main]
#![no_std]

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
)]
mod app {
    //use rtic::cyccnt::U32Ext;

    use audio::AudioBuffer;
    use libdaisy::{audio, gpio::*, hid, logger, prelude::*, system, MILICYCLES};
    use log::info;
    use stm32h7xx_hal::{adc, stm32, time::MilliSeconds, timer::Timer};

    #[shared]
    struct Shared {
        control1: hid::AnalogControl<Daisy15<Analog>>,
    }

    #[local]
    struct Local {
        audio: audio::Audio,
        adc1: adc::Adc<stm32::ADC1, adc::Enabled>,
        timer2: Timer<stm32::TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();
        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::SeedSystem::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let mut system = libdaisy::seed_system_init!(core, device, ccdr);

        info!("Enable adc1");
        let mut adc1 = system.adc1.enable();
        adc1.set_resolution(adc::Resolution::SixteenBit);
        let adc1_max = adc1.slope() as f32;

        let daisy15 = system
            .gpio
            .daisy15
            .take()
            .expect("Failed to get pin daisy15!")
            .into_analog();

        let mut control1 = hid::AnalogControl::new(daisy15, adc1_max);
        // Transform linear input into logarithmic
        control1.set_transform(|x| x * x);

        let timer2 = stm32h7xx_hal::timer::TimerExt::timer(
            device.TIM2,
            MilliSeconds::from_ticks(100).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );

        (
            Shared { control1 },
            Local {
                audio: system.audio,
                adc1,
                timer2,
            },
            init::Monotonics(),
        )
    }

    // Non-default idle ensures chip doesn't go to sleep which causes issues for
    // probe.rs currently
    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // Interrupt handler for audio
    #[task(binds = DMA1_STR1, local = [audio], shared = [control1], priority = 8)]
    fn audio_handler(mut ctx: audio_handler::Context) {
        let audio = ctx.local.audio;

        ctx.shared.control1.lock(|c| {
            let volume = c.get_value();
            info!("{}", volume);

            audio.for_each(|left, right| (left * volume, right * volume));
        });
    }

    #[task(binds = TIM2, local = [timer2, adc1], shared = [control1])]
    fn interface_handler(mut ctx: interface_handler::Context) {
        ctx.local.timer2.clear_irq();
        let adc1 = ctx.local.adc1;

        ctx.shared.control1.lock(|control1| {
            if let Ok(data) = adc1.read(control1.get_pin()) {
                control1.update(data);
            };
        });
    }
}

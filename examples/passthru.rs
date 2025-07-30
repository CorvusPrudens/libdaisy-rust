//! examples/passthru.rs
#![no_main]
#![no_std]
#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
)]
mod app {
    const BLOCK_SIZE: usize = 128;
    use libdaisy::logger;
    use libdaisy::{audio, system};
    use log::info;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        audio: audio::Audio,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();

        // Latest changes here. This approach allows you to
        // access peripherals and resources that were simply
        // moved out of the function in the previous implementation.
        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::SeedSeedSystem::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let system = libdaisy::seed_seed_system_init!(core, device, ccdr, BLOCK_SIZE);

        info!("Startup done!!");

        (
            Shared {},
            Local {
                audio: system.audio,
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
    #[task(binds = DMA1_STR1, local = [audio], priority = 8)]
    fn audio_handler(ctx: audio_handler::Context) {
        let audio = ctx.local.audio;

        audio.for_each(|left, right| (left, right));
    }
}

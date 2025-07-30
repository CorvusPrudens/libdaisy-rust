//! examples/delay.rs
#![no_main]
#![no_std]

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
)]
mod app {
    use libdaisy::{
        audio::{self, AudioBuffer},
        logger, system,
    };
    use log::info;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        audio: audio::Audio,
        buffer: AudioBuffer,
        sdram: &'static mut [f32],
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();

        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::SeedSystem::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let system = libdaisy::seed_system_init!(core, device, ccdr);
        let buffer = [(0.0, 0.0); audio::BLOCK_SIZE_MAX];

        info!("Startup done!");

        (
            Shared {},
            Local {
                audio: system.audio,
                buffer,
                sdram: system.sdram,
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
    #[task(binds = DMA1_STR1, local = [audio, buffer, sdram, index: usize = 0], priority = 8)]
    fn audio_handler(ctx: audio_handler::Context) {
        let audio = ctx.local.audio;
        let buffer = ctx.local.buffer;
        let sdram: &mut [f32] = ctx.local.sdram;
        let index: &mut usize = ctx.local.index;

        if audio.get_stereo(buffer) {
            for (left, right) in buffer {
                audio
                    .push_stereo((sdram[*index], sdram[*index + 1]))
                    .unwrap();
                sdram[*index] = *left;
                sdram[*index + 1] = *right;
                *index = (*index + 2) % libdaisy::AUDIO_SAMPLE_RATE;
            }
        }
    }
}

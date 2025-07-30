//! examples/sine.rs
#![no_main]
#![no_std]

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true
)]
mod app {
    use libdaisy::{
        audio::{self, AudioBuffer},
        gpio::SeedLed,
        logger, system,
    };
    use libm;
    use log::info;

    pub struct AudioRate {
        pub audio: audio::Audio,
        buffer: AudioBuffer,
    }

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        ar: AudioRate,
        phase: f32,
        pitch: f32,
        led: SeedLed,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();
        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::SeedSystem::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let system = libdaisy::seed_system_init!(core, device, ccdr);
        info!("Startup done!");
        let buffer = [(0.0, 0.0); audio::BLOCK_SIZE_MAX];

        (
            Shared {},
            Local {
                ar: AudioRate {
                    audio: system.audio,
                    buffer,
                },
                phase: 0.0,
                pitch: 440.0,
                led: system.gpio.led,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_cx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    // Interrupt handler for audio
    #[task(binds = DMA1_STR1, local = [ar, phase, pitch, led], shared = [], priority = 8)]
    fn audio_handler(ctx: audio_handler::Context) {
        let audio = &mut ctx.local.ar.audio;
        let buffer = &mut ctx.local.ar.buffer;
        let phase = ctx.local.phase;
        let pitch = ctx.local.pitch;
        let led = ctx.local.led;

        audio.get_stereo(buffer);
        for _ in 0..buffer.len() {
            // phase is gonna get bigger and bigger
            // at some point floating point errors will quantize the pitch
            *phase += *pitch / libdaisy::AUDIO_SAMPLE_RATE as f32;
            let mono = libm::sinf(*phase);
            audio.push_stereo((mono, mono)).unwrap();

            if *pitch > 10_000.0 {
                *pitch = 440.0;
                led.toggle();
            }

            *pitch += 0.1;
        }
    }
}

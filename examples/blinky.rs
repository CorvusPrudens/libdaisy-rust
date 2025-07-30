//! examples/blinky.rs
#![no_main]
#![no_std]

#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true
)]
mod app {
    // Includes a panic handler and optional logging facilities
    use libdaisy::{gpio, logger, system};
    use log::info;
    use stm32h7xx_hal::{stm32, time::MilliSeconds, timer::Timer};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        seed_led: gpio::SeedLed,
        timer2: Timer<stm32::TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();
        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::SeedSystem::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let system = libdaisy::seed_system_init!(core, device, ccdr);
        info!("Startup done!");
        let mut timer2 = stm32h7xx_hal::timer::TimerExt::timer(
            device.TIM2,
            MilliSeconds::from_ticks(100).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );
        timer2.listen(stm32h7xx_hal::timer::Event::TimeOut);

        timer2.set_freq(MilliSeconds::from_ticks(500).into_rate());

        (
            Shared {},
            Local {
                seed_led: system.gpio.led,
                timer2,
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

    #[task(binds = TIM2, local = [timer2, seed_led, led_is_on: bool = true])]
    fn blink(ctx: blink::Context) {
        ctx.local.timer2.clear_irq();

        if *ctx.local.led_is_on {
            ctx.local.seed_led.set_high();
        } else {
            ctx.local.seed_led.set_low();
        }
        *ctx.local.led_is_on = !(*ctx.local.led_is_on);
    }
}

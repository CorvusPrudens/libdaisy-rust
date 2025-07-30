#![deny(warnings)]
#![deny(unsafe_code)]
#![no_std]
#![no_main]

#[rtic::app(device = stm32h7xx_hal::stm32, peripherals = true)]
mod app {
    use libdaisy::{
        gpio,
        hal::gpio::{Edge, ExtiPin},
        logger, system,
    };
    use log::info;

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        button: gpio::SeedButton,
        led: gpio::SeedLed,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();
        let mut core = ctx.core;
        let mut device = ctx.device;
        let ccdr = system::SeedSeedSystem::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let mut system = libdaisy::seed_seed_system_init!(core, device, ccdr);
        info!("Startup done!");

        // Button
        system.gpio.button.make_interrupt_source(&mut device.SYSCFG);
        system
            .gpio
            .button
            .trigger_on_edge(&mut device.EXTI, Edge::Rising);
        system.gpio.button.enable_interrupt(&mut device.EXTI);

        (
            Shared {},
            Local {
                button: system.gpio.button,
                led: system.gpio.led,
            },
            init::Monotonics(),
        )
    }

    #[task(binds = EXTI3, local = [button, led])]
    fn button_click(ctx: button_click::Context) {
        ctx.local.button.clear_interrupt_pending_bit();
        ctx.local.led.toggle();
    }
}

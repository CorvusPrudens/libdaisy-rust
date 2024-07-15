//! examples/toggle.rs
#![no_main]
#![no_std]
#[rtic::app(
    device = stm32h7xx_hal::stm32,
    peripherals = true,
)]
mod app {
    // Includes a panic handler and optional logging facilities
    use libdaisy::logger;
    use libdaisy::{gpio::*, hid, prelude::*, system};
    use log::info;
    use stm32h7xx_hal::time::MilliSeconds;
    use stm32h7xx_hal::{stm32, time::Hertz, timer::Timer};

    #[shared]
    struct Shared {}

    #[local]
    struct Local {
        seed_led: SeedLed,
        switch1: hid::Switch<Daisy28<Input>>,
        timer2: Timer<stm32::TIM2>,
    }

    #[init]
    fn init(ctx: init::Context) -> (Shared, Local, init::Monotonics) {
        logger::init();
        let mut core = ctx.core;
        let device = ctx.device;
        let ccdr = system::System::init_clocks(device.PWR, device.RCC, &device.SYSCFG);
        let mut system = libdaisy::system_init!(core, device, ccdr);
        info!("Startup done!");
        let mut timer2 = stm32h7xx_hal::timer::TimerExt::timer(
            device.TIM2,
            MilliSeconds::from_ticks(100).into_rate(),
            ccdr.peripheral.TIM2,
            &ccdr.clocks,
        );

        let daisy28 = system
            .gpio
            .daisy28
            .take()
            .expect("Failed to get pin daisy28!")
            .into_pull_up_input();

        timer2.set_freq(Hertz::from_raw(100));

        let switch1 = hid::Switch::new(daisy28, hid::SwitchType::PullUp);

        (
            Shared {},
            Local {
                seed_led: system.gpio.led,
                switch1,
                timer2,
            },
            init::Monotonics(),
        )
    }

    #[idle]
    fn idle(_ctx: idle::Context) -> ! {
        loop {
            cortex_m::asm::nop();
        }
    }

    #[task(binds = TIM2, local = [timer2, seed_led, switch1, led_is_on: bool = false])]
    fn interface_handler(ctx: interface_handler::Context) {
        ctx.local.timer2.clear_irq();
        let switch1 = ctx.local.switch1;
        switch1.update();

        if switch1.is_falling() {
            info!("Button pressed!");
            if *ctx.local.led_is_on {
                ctx.local.seed_led.set_high();
            } else {
                ctx.local.seed_led.set_low();
            }
            *ctx.local.led_is_on = !(*ctx.local.led_is_on);
        }
    }
}

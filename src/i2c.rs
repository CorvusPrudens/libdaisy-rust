//! Blocking I2C with timeouts.
//!
//! The HAL's blocking transactions can spin forever in certain circumstances.
//! This provides the same functionality with hardcoded 2ms timeouts.

use crate::hal;
use core::ops::Deref;
use cortex_m::peripheral::DWT;
use embedded_hal::blocking::i2c::{Read, Write, WriteRead};
use hal::i2c::I2c;
use hal::pac::i2c1::{isr, RegisterBlock};

/// How long a single step of a transaction may take before giving up.
pub const DEFAULT_TIMEOUT_US: u32 = 2_000;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Error {
    /// The addressed device or a data byte was not acknowledged.
    NotAcknowledge,
    /// A misplaced START or STOP was observed.
    Bus,
    /// Another master won the bus.
    Arbitration,
    /// A step of the transaction never completed. The peripheral has been reset.
    Timeout,
}

/// An I2C master whose transactions always return.
pub struct I2cTimeout<I2C> {
    i2c: I2C,
    timeout_cycles: u32,
}

impl<I2C> I2cTimeout<I2C>
where
    I2C: Deref<Target = RegisterBlock>,
{
    /// Take over a peripheral the HAL has already clocked and timed, e.g.
    /// `I2cTimeout::new(device.I2C1.i2c(pins, freq, rec, &clocks))`.
    pub fn new(i2c: I2c<I2C>) -> Self
    where
        I2c<I2C>: Free<I2C>,
    {
        Self::with_timeout_us(i2c, DEFAULT_TIMEOUT_US)
    }

    pub fn with_timeout_us(i2c: I2c<I2C>, timeout_us: u32) -> Self
    where
        I2c<I2C>: Free<I2C>,
    {
        Self {
            i2c: i2c.free_peripheral(),
            timeout_cycles: timeout_us.saturating_mul(crate::MICROCYCLES),
        }
    }

    fn deadline(&self) -> u32 {
        DWT::cycle_count().wrapping_add(self.timeout_cycles)
    }

    fn expired(deadline: u32) -> bool {
        // Signed distance, so the counter's wrap (~9 s at 480 MHz) is harmless.
        (DWT::cycle_count().wrapping_sub(deadline) as i32) >= 0
    }

    /// Clear PE: the state machine and flags reset and SCL/SDA are released.
    /// PE must stay low for at least three APB cycles.
    fn reset(&mut self) {
        self.i2c.cr1.modify(|_, w| w.pe().clear_bit());
        for _ in 0..3 {
            let _ = self.i2c.cr1.read();
        }
        self.i2c.cr1.modify(|_, w| w.pe().set_bit());
    }

    /// Spin until `done`, an error flag, or the deadline.
    fn wait(&mut self, done: impl Fn(&isr::R) -> bool) -> Result<(), Error> {
        let deadline = self.deadline();
        loop {
            let isr = self.i2c.isr.read();

            if done(&isr) {
                return Ok(());
            } else if isr.berr().is_error() {
                self.i2c.icr.write(|w| w.berrcf().set_bit());
                return Err(Error::Bus);
            } else if isr.arlo().is_lost() {
                self.i2c.icr.write(|w| w.arlocf().set_bit());
                return Err(Error::Arbitration);
            } else if isr.nackf().bit_is_set() {
                // The hardware sends STOP on its own after a NACK.
                self.i2c
                    .icr
                    .write(|w| w.stopcf().set_bit().nackcf().set_bit());
                self.flush_txdr();
                return Err(Error::NotAcknowledge);
            } else if Self::expired(deadline) {
                self.reset();
                return Err(Error::Timeout);
            }
        }
    }

    fn flush_txdr(&mut self) {
        if self.i2c.isr.read().txis().bit_is_set() {
            self.i2c.txdr.write(|w| w.txdata().bits(0));
        }
        if self.i2c.isr.read().txe().is_not_empty() {
            self.i2c.isr.write(|w| w.txe().set_bit());
        }
    }

    fn start(&mut self, addr: u8, length: usize, read: bool, autoend: bool) -> Result<(), Error> {
        // a previous address phase may still be active for up to half a
        // bus cycle
        let deadline = self.deadline();
        while self.i2c.cr2.read().start().bit_is_set() {
            if Self::expired(deadline) {
                self.reset();
                return Err(Error::Timeout);
            }
        }

        self.i2c.cr2.write(|w| {
            let w = w
                .sadd()
                .bits(u16::from(addr) << 1)
                .add10()
                .clear_bit()
                .nbytes()
                .bits(length as u8)
                .start()
                .set_bit()
                .autoend()
                .bit(autoend);
            match read {
                true => w.rd_wrn().read(),
                false => w.rd_wrn().write(),
            }
        });

        Ok(())
    }

    fn send(&mut self, bytes: &[u8]) -> Result<(), Error> {
        for byte in bytes {
            // START (or the previous byte) was acknowledged.
            self.wait(|isr| isr.txis().is_empty())?;
            self.i2c.txdr.write(|w| w.txdata().bits(*byte));
        }
        Ok(())
    }

    fn receive(&mut self, buffer: &mut [u8]) -> Result<(), Error> {
        for byte in buffer {
            self.wait(|isr| isr.rxne().is_not_empty())?;
            *byte = self.i2c.rxdr.read().rxdata().bits();
        }
        Ok(())
    }
}

impl<I2C> Write for I2cTimeout<I2C>
where
    I2C: Deref<Target = RegisterBlock>,
{
    type Error = Error;

    fn write(&mut self, addr: u8, bytes: &[u8]) -> Result<(), Error> {
        assert!(!bytes.is_empty() && bytes.len() < 256);

        self.start(addr, bytes.len(), false, false)?;
        self.send(bytes)?;
        self.wait(|isr| isr.tc().is_complete())?;
        self.i2c.cr2.write(|w| w.stop().set_bit());
        self.wait(|isr| isr.busy().is_not_busy())
    }
}

impl<I2C> WriteRead for I2cTimeout<I2C>
where
    I2C: Deref<Target = RegisterBlock>,
{
    type Error = Error;

    fn write_read(&mut self, addr: u8, bytes: &[u8], buffer: &mut [u8]) -> Result<(), Error> {
        assert!(!bytes.is_empty() && bytes.len() < 256);
        assert!(!buffer.is_empty() && buffer.len() < 256);

        self.start(addr, bytes.len(), false, false)?;
        self.send(bytes)?;
        self.wait(|isr| isr.tc().is_complete())?;
        // Repeated START; the previous address phase is over, so no wait.
        self.i2c.cr2.write(|w| {
            w.sadd()
                .bits(u16::from(addr) << 1 | 1)
                .add10()
                .clear_bit()
                .rd_wrn()
                .read()
                .nbytes()
                .bits(buffer.len() as u8)
                .start()
                .set_bit()
                .autoend()
                .set_bit()
        });
        self.receive(buffer)?;
        self.wait(|isr| isr.busy().is_not_busy())
    }
}

impl<I2C> Read for I2cTimeout<I2C>
where
    I2C: Deref<Target = RegisterBlock>,
{
    type Error = Error;

    fn read(&mut self, addr: u8, buffer: &mut [u8]) -> Result<(), Error> {
        assert!(!buffer.is_empty() && buffer.len() < 256);

        self.start(addr, buffer.len(), true, true)?;
        self.receive(buffer)?;
        self.wait(|isr| isr.busy().is_not_busy())
    }
}

/// The HAL's `I2c::free` is per-instance rather than generic.
pub trait Free<I2C> {
    fn free_peripheral(self) -> I2C;
}

macro_rules! free {
    ($($I2CX:ident),+) => {
        $(
            impl Free<hal::pac::$I2CX> for I2c<hal::pac::$I2CX> {
                fn free_peripheral(self) -> hal::pac::$I2CX {
                    self.free().0
                }
            }
        )+
    };
}

free!(I2C1, I2C2, I2C3, I2C4);

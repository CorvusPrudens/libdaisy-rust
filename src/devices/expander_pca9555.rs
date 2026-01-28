use crate::hal;
use hal::i2c::I2c;
use hal::prelude::*;
use stm32h7xx_hal::prelude::{
    _embedded_hal_blocking_i2c_Write as Write, _embedded_hal_blocking_i2c_WriteRead as WriteRead,
};

/// Data corresponding to a single PCA9555 I/O Expander
///
/// In the future we can add:
/// - chained-device support (similar to pca9685)
/// - output configuration
/// - INT pin support for non-polling behavior
///
/// For now this is a minimal driver for providing access
/// to 16 additional inptus.
pub struct Pca9555 {
    address: u8,
    states: [bool; 16],
}

#[derive(Debug)]
pub enum I2cError<W, WR> {
    Write(W),
    WriteRead(WR),
}

#[expect(type_alias_bounds)]
pub type WriteReadError<W: Write + WriteRead> =
    I2cError<<W as Write>::Error, <W as WriteRead>::Error>;

impl Pca9555 {
    pub fn new<W: Write + WriteRead>(address: u8, bus: &mut W) -> Result<Self, WriteReadError<W>> {
        let base_address = 0x20;
        let dev_addr = base_address + address;
        let dev = Pca9555 {
            address: dev_addr,
            states: Default::default(),
        };

        // Configuration
        // Default config appears to be:
        // configuration = input
        // polarity = not-inverted
        // So for our initial implementation.. no config is necessary

        // Fill internal data with current states on hardware.
        dev.update();

        Ok(dev)
    }

    /// Polls the device, reading both of it's input registers to fill the
    /// internal state data
    pub fn update<W: Write + WriteRead>(&mut self, bus: &mut W) -> Result<(), WriteReadError<W>> {
        // Register pairs flip automatically after reads.
        // So we can basically just send the `InputPort0` command,
        // and read two data bytes back every time we want new data
        let mut raw_states = [u8; 2];
        self.read(bus, Register::InputPort0, raw_states)?;

        for i in 0..16 {
            let raw_union: u16 = ((raw_states[0] as u16) | ((raw_states as u16) << 8));
            self.states[i] = (raw_union & (1 << i)) != 0;
        }
        Ok(())
    }

    /// Returns the current state recorded for a particular pin
    pub fn get_state(self, index: u16) -> bool {
        let idx = index.clamp(0, 16);
        self.states[idx]
    }

    fn write<W: Write + WriteRead>(
        &self,
        bus: &mut W,
        register: Register,
        data: u8,
    ) -> Result<(), WriteReadError<W>> {
        self.write_raw(bus, register as u8, data)
    }

    fn write_raw<W: Write + WriteRead>(
        &self,
        bus: &mut W,
        register: u8,
        data: u8,
    ) -> Result<(), WriteReadError<W>> {
        // This is probably not what we need
        bus.write(self.address, &[register, data])
            .map_err(I2cError::Write)?;

        Ok(())
    }

    fn read<W: Write + WriteRead>(
        &self,
        bus: &mut W,
        register: Register,
        buffer: &mut [u8],
    ) -> Result<(), WriteReadError<W>> {
        self.read_raw(bus, register as u8, buffer)
    }

    fn read_raw<W: Write + WriteRead>(
        &self,
        bus: &mut W,
        register: u8,
        buffer: &mut [u8],
    ) -> Result<(), WriteReadError<W>> {
        bus.write_read(self.address, &[register], buffer)
            .map_err(I2cError::WriteRead)
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
enum Register {
    InputPort0 = 0x00,
    InputPort1 = 0x01,
    OutputPort0 = 0x02,
    OutputPort1 = 0x03,
    PolarityInversion0 = 0x04,
    PolarityInversion1 = 0x05,
    ConfigurationPort0 = 0x06,
    ConfigurationPort1 = 0x07,
}

use crate::hal;
use hal::i2c::I2c;
use hal::prelude::*;
use stm32h7xx_hal::prelude::{
    _embedded_hal_blocking_i2c_Read as Read, _embedded_hal_blocking_i2c_Write as Write,
    _embedded_hal_blocking_i2c_WriteRead as WriteRead,
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
pub struct Pca9555<I2CN> {
    i2c: I2c<I2CN>,
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

impl<I2CN> Pca9555<I2CN>
where
    I2c<I2CN>: Write + WriteRead + Read,
{
    pub fn new(address: u8, mut bus: I2c<I2CN>) -> Self {
        let base_address = 0x20;
        let dev_addr = base_address | (address & 0x07);
        let mut dev = Pca9555 {
            i2c: bus,
            address: dev_addr,
            states: Default::default(),
        };

        // Configuration
        // Default config appears to be:
        // configuration = input
        // polarity = not-inverted
        // So for our initial implementation.. no config is necessary

        // Any attempts to write here will cause an infinite loop during if device is not
        // present and serial callback hasn't been started. So maybe we don't do this here yet.

        // This was here to test why device wasn't sending ACK (but it wasn't on the PCB ...)
        // Will remove these comments, and finish driver once I have appropriate HW
        // crate::delay::CycleDelay::new().delay_ms(30u8);
        // let _ = dev
        //     .i2c
        //     .write(dev.address, &[Register::ConfigurationPort0 as u8, 0xff]);

        // Fill internal data with current states on hardware.
        // dev.update();

        dev
    }

    /// Polls the device, reading both of it's input registers to fill the
    /// internal state data
    // pub fn update(&mut self) -> Result<(), WriteReadError<W>> {
    pub fn update(&mut self) {
        // Register pairs flip automatically after reads.
        // So we can basically just send the `InputPort0` command,
        // and read two data bytes back every time we want new data
        let mut raw_states = [0u8; 2];
        let _ = self
            .i2c
            .write_read(self.address, &[Register::InputPort0 as u8], &mut raw_states);

        let raw_union: u16 = (raw_states[0] as u16) | ((raw_states[1] as u16) << 8);
        for i in 0..16 {
            self.states[i] = (raw_union & (1 << i)) != 0;
        }
    }

    /// Returns the current state recorded for a particular pin
    pub fn get_state(&self, index: usize) -> bool {
        let idx = index.min(15);
        self.states[idx]
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

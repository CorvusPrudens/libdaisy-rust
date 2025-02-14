use cortex_m::prelude::_embedded_hal_blocking_delay_DelayMs;
use stm32h7xx_hal::prelude::{
    _embedded_hal_blocking_i2c_Write as Write, _embedded_hal_blocking_i2c_WriteRead as WriteRead,
};

pub struct Mpr121 {
    address: u8,
    thresholds: [Mpr121Thresholds; 12],
    states: TouchStates,
    analog_states: [f32; 12],
}

#[derive(Debug)]
pub enum I2cError<W, WR> {
    Write(W),
    WriteRead(WR),
}

#[expect(type_alias_bounds)]
pub type WriteReadError<W: Write + WriteRead> =
    I2cError<<W as Write>::Error, <W as WriteRead>::Error>;

impl Mpr121 {
    pub fn new<W: Write + WriteRead>(
        address: u8,
        touch_threshold: u8,
        release_threshold: u8,
        bus: &mut W,
    ) -> Result<Self, WriteReadError<W>> {
        let mpr = Mpr121 {
            address,
            thresholds: [Mpr121Thresholds {
                touch: touch_threshold,
                release: release_threshold,
            }; 12],
            states: Default::default(),
            analog_states: [0.0; 12],
        };

        mpr.write(bus, Register::SoftReset, 0x63)?;
        crate::delay::CycleDelay::new().delay_ms(1u8);

        mpr.write(bus, Register::Ecr, 0x0)?;

        mpr.write(bus, Register::Mhdr, 0x01)?;
        mpr.write(bus, Register::Nhdr, 0x01)?;
        mpr.write(bus, Register::Nclr, 0x0E)?;
        mpr.write(bus, Register::Fdlr, 0x00)?;

        mpr.write(bus, Register::Mhdf, 0x01)?;
        mpr.write(bus, Register::Nhdf, 0x05)?;
        mpr.write(bus, Register::Nclf, 0x01)?;
        mpr.write(bus, Register::Fdlf, 0x00)?;

        mpr.write(bus, Register::Nhdt, 0x00)?;
        mpr.write(bus, Register::Nclt, 0x00)?;
        mpr.write(bus, Register::Fdlt, 0x00)?;

        mpr.write(bus, Register::Debounce, 0)?;
        mpr.write(bus, Register::Config1, 0x10)?; // default, 16uA charge current
        mpr.write(bus, Register::Config2, 0x20)?; // 0.5uS encoding, 1ms period

        let ecr_settings = 0x80 + 12; // 5 bits for baseline tracking & proximity disabled + X
                                      // amount of electrodes running (12)
        mpr.write(bus, Register::Ecr, ecr_settings)?; // start with above ECR setting

        mpr.write_thresholds(bus)?;

        Ok(mpr)
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
        // first get the current set value of the MPR121_ECR register
        let ecr_reg = Register::Ecr as u8;
        let mut ecr_backup = 0;

        bus.write_read(
            self.address,
            &[ecr_reg],
            core::slice::from_mut(&mut ecr_backup),
        )
        .map_err(I2cError::WriteRead)?;

        // MPR121 must be put in Stop Mode to write to most registers
        let stop_required = !((register == ecr_reg) || (0x73..=0x7A).contains(&register));

        if stop_required {
            // clear this register to set stop mode
            bus.write(self.address, &[ecr_reg, 0x00])
                .map_err(I2cError::Write)?;
        }

        bus.write(self.address, &[register, data])
            .map_err(I2cError::Write)?;

        if stop_required {
            // write back the previous set ECR settings
            bus.write(self.address, &[ecr_reg, ecr_backup])
                .map_err(I2cError::Write)?;
        }

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

    pub fn write_thresholds<W: Write + WriteRead>(
        &self,
        bus: &mut W,
    ) -> Result<(), WriteReadError<W>> {
        for (i, Mpr121Thresholds { touch, release }) in self.thresholds.iter().enumerate() {
            let touch_register = Register::TouchTh0 as usize + i * 2;
            let release_register = Register::ReleaseTh0 as usize + i * 2;

            self.write_raw(bus, touch_register as u8, *touch)?;
            self.write_raw(bus, release_register as u8, *release)?;
        }

        Ok(())
    }

    pub fn update<W: Write + WriteRead>(&mut self, bus: &mut W) -> Result<(), WriteReadError<W>> {
        let mut buffer = [0u8; 2];
        self.read(bus, Register::TouchStatusL, &mut buffer)?;

        self.states = TouchStates(u16::from_le_bytes(buffer));
        let u10_scale = 1.0 / (1 << 10) as f32;

        for i in 0..self.analog_states.len() {
            let register = Register::FiltData0L as usize + i * 2;
            self.read_raw(bus, register as u8, &mut buffer)?;

            // the data returned is a 10-bit unsigned value
            self.analog_states[i] = u16::from_le_bytes(buffer) as f32 * u10_scale;
        }

        Ok(())
    }

    pub fn states(&self) -> TouchStates {
        self.states
    }

    pub fn analog_states(&self) -> [f32; 12] {
        self.analog_states
    }
}

#[derive(Default, Clone, Copy)]
pub struct TouchStates(u16);

impl TouchStates {
    pub fn is_touched(&self, channel: u8) -> bool {
        ((self.0 >> channel) & 1) > 0
    }

    pub fn raw(&self) -> u16 {
        self.0
    }
}

#[derive(Clone, Copy)]
pub struct Mpr121Thresholds {
    pub touch: u8,
    pub release: u8,
}

impl Default for Mpr121Thresholds {
    fn default() -> Self {
        Mpr121Thresholds {
            touch: 12,
            release: 6,
        }
    }
}

#[derive(Debug, Copy, Clone, PartialEq, Eq)]
#[repr(u8)]
enum Register {
    TouchStatusL = 0x00,
    TouchStatusH = 0x01,
    FiltData0L = 0x04,
    FiltData0H = 0x05,
    Baseline0 = 0x1E,
    Mhdr = 0x2B,
    Nhdr = 0x2C,
    Nclr = 0x2D,
    Fdlr = 0x2E,
    Mhdf = 0x2F,
    Nhdf = 0x30,
    Nclf = 0x31,
    Fdlf = 0x32,
    Nhdt = 0x33,
    Nclt = 0x34,
    Fdlt = 0x35,

    TouchTh0 = 0x41,
    ReleaseTh0 = 0x42,
    Debounce = 0x5B,
    Config1 = 0x5C,
    Config2 = 0x5D,
    ChargeCurr0 = 0x5F,
    ChargeTime1 = 0x6C,
    Ecr = 0x5E,
    AutoConfig0 = 0x7B,
    AutoConfig1 = 0x7C,
    UpLimit = 0x7D,
    LowLimit = 0x7E,
    TargetLimit = 0x7F,

    GpioDir = 0x76,
    GpioEn = 0x77,
    GpioSet = 0x78,
    GpioClr = 0x79,
    GpioToggle = 0x7A,

    SoftReset = 0x80,
}

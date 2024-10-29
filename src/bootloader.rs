use core::mem::MaybeUninit;
use cortex_m::prelude::*;
use stm32h7xx_hal::{delay::Delay, pac::SCB};

#[link_section = ".boot_info"]
static mut BOOT_INFO: MaybeUninit<[u32; 3]> = MaybeUninit::uninit();

unsafe fn get_info() -> [u32; 3] {
    let info_ptr = core::ptr::addr_of!(BOOT_INFO);
    unsafe { core::ptr::read_volatile(info_ptr).assume_init() }
}

unsafe fn set_info(info: [u32; 3]) {
    let info_ptr = core::ptr::addr_of_mut!(BOOT_INFO);
    unsafe {
        core::ptr::write_volatile(info_ptr, MaybeUninit::new(info));
    }
}

// These should probably be defined in the linker,
// but this'll do for now.
const D1_AXIFLASH_BASE: u32 = 0x0800_0000;
const D1_ITCMRAM_BASE: u32 = 0x0000_0000;
const D1_DTCMRAM_BASE: u32 = 0x2000_0000;
pub const D1_AXISRAM_BASE: u32 = 0x2400_0000;
const D2_AXISRAM_BASE: u32 = 0x3000_0000;
const D3_SRAM_BASE: u32 = 0x3800_0000;
const SDRAM_BASE: u32 = 0xC000_0000;
const QSPI_BASE: u32 = 0x9000_0000;

const INTERNAL_FLASH_SIZE: u32 = 0x20000;
const ITCMRAM_SIZE: u32 = 0x10000;
const DTCMRAM_SIZE: u32 = 0x20000;
pub const SRAM_D1_SIZE: u32 = 0x80000;
const SRAM_D2_SIZE: u32 = 0x48000;
const SRAM_D3_SIZE: u32 = 0x10000;
const SDRAM_SIZE: u32 = 0x4000000;
const QSPI_SIZE: u32 = 0x800000;

const D1_AXIFLASH_END: u32 = D1_AXIFLASH_BASE + INTERNAL_FLASH_SIZE - 1;
const D1_ITCMRAM_END: u32 = D1_ITCMRAM_BASE + ITCMRAM_SIZE - 1;
const D1_DTCMRAM_END: u32 = D1_DTCMRAM_BASE + DTCMRAM_SIZE - 1;
const D1_AXISRAM_END: u32 = D1_AXISRAM_BASE + SRAM_D1_SIZE - 1;
const D2_AXISRAM_END: u32 = D2_AXISRAM_BASE + SRAM_D2_SIZE - 1;
const D3_SRAM_END: u32 = D3_SRAM_BASE + SRAM_D3_SIZE - 1;
const SDRAM_END: u32 = SDRAM_BASE + SDRAM_SIZE - 1;
const QSPI_END: u32 = QSPI_BASE + QSPI_SIZE - 1;

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MemoryRegion {
    InternalFlash,
    ItcmRam,
    DtcmRam,
    SramD1,
    SramD2,
    SramD3,
    SdRam,
    Qspi,
}

impl MemoryRegion {
    /// Return the memory region containing this address, if any.
    pub fn from_address(address: u32) -> Option<Self> {
        match address {
            D1_AXIFLASH_BASE..=D1_AXIFLASH_END => Some(MemoryRegion::InternalFlash),
            D1_ITCMRAM_BASE..=D1_ITCMRAM_END => Some(MemoryRegion::ItcmRam),
            D1_DTCMRAM_BASE..=D1_DTCMRAM_END => Some(MemoryRegion::DtcmRam),
            D1_AXISRAM_BASE..=D1_AXISRAM_END => Some(MemoryRegion::SramD1),
            D2_AXISRAM_BASE..=D2_AXISRAM_END => Some(MemoryRegion::SramD2),
            D3_SRAM_BASE..=D3_SRAM_END => Some(MemoryRegion::SramD3),
            SDRAM_BASE..=SDRAM_END => Some(MemoryRegion::SdRam),
            QSPI_BASE..=QSPI_END => Some(MemoryRegion::Qspi),
            _ => None,
        }
    }

    /// Return the memory region occupied by the vector table,
    /// which is a proxy for determining the execution location of the
    /// current program.
    pub fn from_vtable(scb: &SCB) -> Option<Self> {
        let address = scb.vtor.read();

        Self::from_address(address)
    }
}

/// Describes how the Daisy bootloader should behave.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum DaisyBootType {
    /// This is what the bootloader writes when it resets itself.
    ///
    /// TODO: document better
    Jump,
    /// Jump immediately to the target application.
    SkipTimeout,
    /// Wait in the bootloader indefinitely.
    InfiniteTimeout,
}

impl DaisyBootType {
    /// Read from backup SRAM to get the boot type, if any.
    pub fn from_backup() -> Option<Self> {
        let info = unsafe { get_info() };

        let boot_word = info[0];

        Self::from_bits(boot_word)
    }

    fn to_bits(&self) -> u32 {
        match self {
            Self::Jump => 0xDEADBEEF,
            Self::SkipTimeout => 0x5AFEB007,
            Self::InfiniteTimeout => 0xB0074EFA,
        }
    }

    fn from_bits(bits: u32) -> Option<Self> {
        match bits {
            0xDEADBEEF => Some(Self::Jump),
            0x5AFEB007 => Some(Self::SkipTimeout),
            0xB0074EFA => Some(Self::InfiniteTimeout),
            _ => None,
        }
    }

    /// Write the boot type to backup SRAM.
    unsafe fn write(&self) {
        let mut info = unsafe { get_info() };
        info[0] = self.to_bits();
        unsafe {
            set_info(info);
        }
    }

    /// Write an invalid bit pattern to backup SRAM, effectively clearing the boot type.
    pub unsafe fn clear() {
        let mut info = unsafe { get_info() };
        info[0] = 0;
        unsafe {
            set_info(info);
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum Version {
    /// Any version below v6.
    LessThanV6,
    /// Exactly v6.
    V6,
    /// v6.1 or above.
    V6_1,
}

impl Version {
    /// Read from backup SRAM to get the version, if any.
    pub fn from_backup() -> Option<Self> {
        let info = unsafe { get_info() };
        let version_word = info[2];

        Self::from_bits(version_word)
    }

    fn from_bits(bits: u32) -> Option<Self> {
        match bits {
            0 => Some(Self::LessThanV6),
            1 => None,
            2 => Some(Self::V6),
            _ => Some(Self::V6_1),
        }
    }

    fn to_bits(&self) -> u32 {
        match self {
            Self::LessThanV6 => 0,
            Self::V6 => 2,
            Self::V6_1 => 3,
        }
    }

    /// Write the version to backup SRAM.
    unsafe fn write(&self) {
        let mut info = unsafe { get_info() };
        info[2] = self.to_bits();
        unsafe {
            set_info(info);
        }
    }
}

/// Get the target application entry point, if present.
///
/// This will only return `Some` when the boot type is `Jump`.
pub fn application_address() -> Option<u32> {
    let boot_type = DaisyBootType::from_backup();

    match boot_type {
        Some(DaisyBootType::Jump) => {
            let info = unsafe { get_info() };
            Some(info[1])
        }
        _ => None,
    }
}

/// Set the target application entry point.
///
/// This also sets the boot type to `Jump`.
pub unsafe fn set_application_address(address: u32) {
    let boot_type = DaisyBootType::Jump;
    unsafe { boot_type.write() };

    let mut info = unsafe { get_info() };
    info[1] = address;
    unsafe {
        set_info(info);
    }
}

/// Describes which bootloader the device will reset to.
#[derive(Debug)]
pub enum BootType {
    /// The built-in ST Micro system bootloader.
    Stm(crate::gpio::SeedButton),
    /// The daisy bootloader with its associated boot type.
    Daisy(DaisyBootType),
}

/// Reset to the provided bootloader.
///
/// # Panics
///
/// Panics if the Daisy bootloader is selected but
/// the program is running from internal flash, meaning
/// no Daisy bootloader can be present.
pub fn reset_to_bootloader(boot_type: BootType, scb: &SCB, delay: &mut Delay) -> ! {
    match boot_type {
        BootType::Stm(boot_pin) => {
            let mut pin = boot_pin.into_push_pull_output();
            pin.set_high();

            // Allow capacitor to charge.
            delay.delay_ms(10u8);
        }
        BootType::Daisy(daisy_type) => {
            let region = MemoryRegion::from_vtable(scb);

            if matches!(region, Some(MemoryRegion::InternalFlash)) {
                panic!("Attempted to jump to non-existent Daisy bootloader");
            }

            unsafe {
                // write twice just in case
                daisy_type.write();
                daisy_type.write();
            }
        }
    }

    system_reset()
}

/// Reset the device.
///
/// This should behave exactly like pressing the RESET button.
pub fn system_reset() -> ! {
    // Clear all interrupts.
    let ptr = unsafe { &*stm32h7xx_hal::pac::RCC::PTR };
    ptr.cier.reset();

    SCB::sys_reset()
}

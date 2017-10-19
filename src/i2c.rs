#![allow(dead_code)]
extern crate stm32f103xx;
use core::cmp::max;
use stm32f103xx::{i2c1, rcc, RCC};

pub const EVENT_MASTER_STARTED: u32 = 0x00030001; /* BUSY, MSL and SB flag */
pub const EVENT_MASTER_TRANSMITTER_MODE_SELECTED: u32 = 0x00070082;  /* BUSY, MSL, ADDR, TXE and TRA flags */
pub const EVENT_MASTER_RECEIVER_MODE_SELECTED: u32 = 0x00030002;  /* BUSY, MSL and ADDR flags */
pub const EVENT_MASTER_BYTE_RECEIVED: u32 = 0x00030040;  /* BUSY, MSL and RXNE flags */
pub const EVENT_MASTER_BYTE_TRANSMITTING: u32 = 0x00070080; /* TRA, BUSY, MSL, TXE flags */
pub const EVENT_MASTER_BYTE_TRANSMITTED: u32 = 0x00070084;  /* TRA, BUSY, MSL, TXE and BTF flags */

const FLAGS_MASK: u32 = 0x00ffffff;
const HSI_VALUE: u32 = 8000000;
const HSE_VALUE: u32 = 8000000;

pub struct I2C<'a> (pub &'a i2c1::RegisterBlock);

pub enum TransDir {
    TRANSMITTER,
    RECEIVER
}

pub enum DutyType {
    DUTY0,
    DUTY1
}

impl<'a> I2C<'a> {
    #[inline(always)]
    fn get_pclk1(rcc: &RCC) -> u32 {
        use stm32f103xx::rcc::cfgr::{SWSR, PLLSRCR, PLLXTPRER};
        let cfgr = rcc.cfgr.read();
        let sysclk_freq = match cfgr.sws() {
            SWSR::HSI => HSI_VALUE,
            SWSR::HSE => HSE_VALUE,
            SWSR::PLL => {
                let pllmull = cfgr.pllmul().bits();
                let pllsource = cfgr.pllsrc();
                let pllmull = pllmull as u32 + 2;
                match pllsource {
                    PLLSRCR::INTERNAL => {
                        (HSI_VALUE >> 1) * pllmull
                    },
                    PLLSRCR::EXTERNAL => {
                        match cfgr.pllxtpre() {
                            PLLXTPRER::DIV2 => (HSE_VALUE >> 1) * pllmull,
                            PLLXTPRER::DIV1 => HSE_VALUE * pllmull
                        }
                    }
                }
            }
            _ => HSI_VALUE
        };
        let div_table: [u8; 16] = [0, 0, 0, 0, 1, 2, 3, 4, 1, 2, 3, 4, 6, 7, 8, 9];
        let hclk_freq = sysclk_freq >> div_table[cfgr.hpre().bits() as usize];
        let pclk1_freq = hclk_freq >> div_table[cfgr.ppre1().bits() as usize];
        pclk1_freq
    }

    /// TODO: support for standard mode
    pub fn init(&self,
                rcc: &RCC,
                addr: u8,
                scl_freq: u32,
                duty_type: DutyType,
                fast_mode: bool) {
        let &I2C(ref i2c) = self;
        unsafe {
            let pclk1 = I2C::get_pclk1(rcc);
            let freq_range: u16 = (pclk1 / 1_000_000) as u16;
            self.pe(false);
            /* TRISE configuration (in Fm mode, max rise interval is 300) */
            i2c.trise.write(|w| w.bits(if fast_mode {(freq_range * 300) / 1000 + 1}
                                       else {freq_range + 1} as u32));
            /* CCR configuration */
            i2c.ccr.write(|w|
                if fast_mode {
                    match duty_type {
                        DutyType::DUTY0 => w.ccr().bits(max(pclk1 / (scl_freq * (2 + 1)), 0x1) as u16),
                        DutyType::DUTY1 => w.ccr().bits(max(pclk1 / (scl_freq * (16 + 9)), 0x1) as u16)
                                            .duty().set_bit(),
                    }.f_s().set_bit()
                } else {
                    w.ccr().bits(max(pclk1 / (scl_freq * (1 + 1)), 0x4) as u16)
                     .f_s().clear_bit()
                });
            self.pe(true); /* PE = 1, enable I2C */
            /* CR1 configuration */
            i2c.cr1.modify(|r, w| w.bits(r.bits())
                                   .smbus().clear_bit()
                                   .smbtype().clear_bit()
                                   .ack().set_bit());
            /* CR2 configuration */
            i2c.cr2.modify(|r, w| w.bits(r.bits()).freq().bits(freq_range as u8));
            /* OAR1 configuration */
            i2c.oar1.write(|w| w.addmode().clear_bit().add7().bits(addr));
            while i2c.sr2.read().busy().bit() {} /* wait until the bus is free */
        }
    }

    pub fn pe(&self, enable: bool) {
        let &I2C(ref i2c) = self;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).pe().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).pe().clear_bit())
            }
        }
    }

    pub fn is_ack_fail(&self) -> bool {
        self.0.sr1.read().af().bit_is_set()
    }

    pub fn start(&self, enable: bool, synced: bool) {
        let &I2C(ref i2c) = self;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).start().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).start().clear_bit())
            }
        }
        if synced {
            match enable {
                true => while !self.check_event(EVENT_MASTER_STARTED) {},
                false => while self.check_event(EVENT_MASTER_STARTED) {}
            }
        }
    }

    pub fn stop(&self, enable: bool) {
        let &I2C(ref i2c) = self;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).stop().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).stop().clear_bit())
            }
        }
    }

    pub fn conf_ack(&self, enable: bool) {
        let &I2C(ref i2c) = self;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).ack().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).ack().clear_bit())
            }
        }
    }

    pub fn send_addr(&self, addr: u8, d: TransDir, synced: bool) -> bool {
        let addr = (addr << 1) | match d {
            TransDir::TRANSMITTER => 0,
            TransDir::RECEIVER => 1
        };
        unsafe {
            self.0.sr1.write(|w| w.af().clear_bit());
            self.0.dr.write(|w| w.dr().bits(addr));
        }
        if synced {
            match d {
                TransDir::TRANSMITTER =>
                    while !self.check_event(EVENT_MASTER_TRANSMITTER_MODE_SELECTED) {
                        if self.is_ack_fail() {
                            return false
                        }
                    },
                TransDir::RECEIVER =>
                    while !self.check_event(EVENT_MASTER_RECEIVER_MODE_SELECTED) {
                        if self.is_ack_fail() {
                            return false
                        }
                    }
            }
        }
        true
    }

    pub fn send(&self, data: u8, synced: bool) {
        unsafe {
            self.0.dr.write(|w| w.dr().bits(data));
        }
        if synced {
            while !self.check_event(EVENT_MASTER_BYTE_TRANSMITTED) {}
        }
    }

    pub fn recv(&self, synced: bool) -> u8 {
        if synced {
            while !self.check_event(EVENT_MASTER_BYTE_RECEIVED) {}
        }
        self.0.dr.read().dr().bits()
    }

    pub fn check_event(&self, ev_mask: u32) -> bool {
        let flags = self.0.sr1.read().bits() & 0xffff |
                    ((self.0.sr2.read().bits() & 0xffff) << 16) & FLAGS_MASK;
        (flags & ev_mask) == ev_mask
    }
}

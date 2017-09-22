#![allow(dead_code)]
extern crate stm32f103xx;
use stm32f103xx::RCC;


pub const EVENT_MASTER_STARTED: u32 = 0x00030001; /* BUSY, MSL and SB flag */
pub const EVENT_MASTER_TRANSMITTER_MODE_SELECTED: u32 = 0x00070082;  /* BUSY, MSL, ADDR, TXE and TRA flags */
pub const EVENT_MASTER_RECEIVER_MODE_SELECTED: u32 = 0x00030002;  /* BUSY, MSL and ADDR flags */
pub const EVENT_MASTER_BYTE_RECEIVED: u32 = 0x00030040;  /* BUSY, MSL and RXNE flags */
pub const EVENT_MASTER_BYTE_TRANSMITTING: u32 = 0x00070080; /* TRA, BUSY, MSL, TXE flags */
pub const EVENT_MASTER_BYTE_TRANSMITTED: u32 = 0x00070084;  /* TRA, BUSY, MSL, TXE and BTF flags */

const FLAGS_MASK: u32 = 0x00ffffff;
const HSI_VALUE: u32 = 8000000;
const HSE_VALUE: u32 = 8000000;
const CCR_CCR_SET: u16 = 0x0FFF;
const CCR_FS_SET: u16 = 0x8000;
const CR1_CLEAR_MASK: u16 = 0xFBF5;
const I2C_ACK_ENABLE: u16 = 0x0400;

pub struct I2C<'a> {
    i2c: &'a stm32f103xx::i2c1::RegisterBlock,
    rcc: &'a stm32f103xx::rcc::RegisterBlock,
}

pub enum TransDir {
    TRANSMITTER,
    RECEIVER
}

impl<'a> I2C<'a> {
    pub fn new(i2c_reg: &'a stm32f103xx::i2c1::RegisterBlock,
               rcc_reg: &'a stm32f103xx::rcc::RegisterBlock) -> I2C<'a> {
        I2C{i2c: i2c_reg, rcc: rcc_reg}
    }

    fn get_pclk1(&self) -> u32 {
        use stm32f103xx::rcc::cfgr::{SWSR, PLLSRCR, PLLXTPRER};
        let cfgr = self.rcc.cfgr.read();
        let sysclk_freq = match cfgr.sws() {
            SWSR::HSI => HSI_VALUE,
            SWSR::HSE => HSE_VALUE,
            SWSR::PLL => {
                let pllmull = cfgr.pllmul().bits();
                let pllsource = cfgr.pllsrc();
                let pllmull = (pllmull as u32 >> 18) + 2;
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

    /// TODO: support for standard mode (100khz)
    pub fn init(&self) {
        let i2c = &self.i2c;
        unsafe {
            self.rcc.apb1rstr.modify(|_, w| w.i2c1rst().set_bit());
            self.rcc.apb1rstr.modify(|_, w| w.i2c1rst().clear_bit());
            self.pe(true); /* PE = 1, enable I2C */
            /* CR2 configuration */
            let pclk1 = self.get_pclk1();
            let freq_range: u16 = (pclk1 / 1000000) as u16;
            i2c.cr2.modify(|r, w| w.bits(r.bits()).freq().bits(freq_range as u8));
            /* CCR configuration */
            self.pe(false);
            let mut res = (pclk1 / (400000 * 3)) as u16;
            if (res & CCR_CCR_SET) == 0 {
                res |= 0x0001;
            }
            /* TRISE configuration */
            i2c.trise.write(|w| w.bits(((freq_range * 300) / 1000 + 1) as u32));
            i2c.ccr.write(|w| w.bits((res | CCR_FS_SET) as u32));
            self.pe(true); /* PE = 1, enable I2C */
            /* CR1 configuration */
            i2c.cr1.modify(|r, w| w.bits(((r.bits() as u16 & CR1_CLEAR_MASK) | I2C_ACK_ENABLE) as u32));
            /* OAR1 configuration */
            i2c.oar1.write(|w| w.addmode().clear_bit().add7().bits(0x01));
            while i2c.sr2.read().busy().bit() {} /* wait until the bus is free */
        }
    }

    pub fn pe(&self, enable: bool) {
        let i2c = &self.i2c;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).pe().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).pe().clear_bit())
            }
        }
    }

    pub fn start(&self, enable: bool, synced: bool) {
        let i2c = &self.i2c;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).start().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).start().clear_bit())
            }
        }
        if synced {
            while !self.check_event(EVENT_MASTER_STARTED) {}
        }
    }

    pub fn stop(&self, enable: bool) {
        let i2c = &self.i2c;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).stop().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).stop().clear_bit())
            }
        }
    }

    pub fn conf_ack(&self, enable: bool) {
        let i2c = &self.i2c;
        unsafe {
            match enable {
                true => i2c.cr1.modify(|r, w| w.bits(r.bits()).ack().set_bit()),
                false => i2c.cr1.modify(|r, w| w.bits(r.bits()).ack().clear_bit())
            }
        }
    }

    pub fn send_addr(&self, addr: u8, d: TransDir, synced: bool) {
        let addr = (addr << 1) | match d {
            TransDir::TRANSMITTER => 0,
            TransDir::RECEIVER => 1
        };
        unsafe {
            self.i2c.dr.write(|w| w.dr().bits(addr));
        }
        if synced {
            match d {
                TransDir::TRANSMITTER =>
                    while !self.check_event(EVENT_MASTER_TRANSMITTER_MODE_SELECTED) {},
                TransDir::RECEIVER =>
                    while !self.check_event(EVENT_MASTER_RECEIVER_MODE_SELECTED) {}
            }
        }
    }

    pub fn send(&self, data: u8, synced: bool) {
        unsafe {
            self.i2c.dr.write(|w| w.dr().bits(data));
        }
        if synced {
            while !self.check_event(EVENT_MASTER_BYTE_TRANSMITTED) {}
        }
    }

    pub fn recv(&self, synced: bool) -> u8 {
        if synced {
            while !self.check_event(EVENT_MASTER_BYTE_RECEIVED) {}
        }
        self.i2c.dr.read().dr().bits()
    }

    pub fn check_event(&self, ev_mask: u32) -> bool {
        let flags = self.i2c.sr1.read().bits() & 0xffff |
                    ((self.i2c.sr2.read().bits() & 0xffff) << 16) & FLAGS_MASK;
        (flags & ev_mask) == ev_mask
    }
}

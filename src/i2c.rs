#![allow(dead_code)]
extern crate stm32f103xx;

pub const EVENT_MASTER_STARTED: u32 = 0x00030001; /* BUSY, MSL and SB flag */
pub const EVENT_MASTER_TRANSMITTER_MODE_SELECTED: u32 = 0x00070082;  /* BUSY, MSL, ADDR, TXE and TRA flags */
pub const EVENT_MASTER_RECEIVER_MODE_SELECTED: u32 = 0x00030002;  /* BUSY, MSL and ADDR flags */
pub const EVENT_MASTER_BYTE_RECEIVED: u32 = 0x00030040;  /* BUSY, MSL and RXNE flags */
pub const EVENT_MASTER_BYTE_TRANSMITTING: u32 = 0x00070080; /* TRA, BUSY, MSL, TXE flags */
pub const EVENT_MASTER_BYTE_TRANSMITTED: u32 = 0x00070084;  /* TRA, BUSY, MSL, TXE and BTF flags */

const CR1_START_MASK: u16 = 0x0100;
const CR1_STOP_MASK: u16 = 0x0200;
const FLAGS_MASK: u32 = 0x00ffffff;

pub struct I2C<'a> {
    i2c: &'a stm32f103xx::i2c1::RegisterBlock,
}

pub enum TransDir {
    TRANSMITTER,
    RECEIVER
}

impl<'a> I2C<'a> {
    pub fn new(reg: &'a stm32f103xx::i2c1::RegisterBlock) -> I2C<'a> {
        I2C{i2c: reg}
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

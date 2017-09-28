extern crate stm32f103xx;
use i2c::{I2C, TransDir, DutyType};

const AT24C_ADDR: u8 = 0b1010111; /* suppose A0, A1, A2 = 1 */

pub struct AT24C<'a, 'b: 'a>(&'a I2C<'a, 'b>);

impl<'a, 'b> AT24C<'a, 'b> {
    pub fn new(i2c: &'a I2C<'a, 'b>) -> AT24C<'a, 'b> {
        AT24C(i2c)
    }

    pub fn read(&self, start: u16, size: usize, buf: &mut [u8]) {
        let &AT24C(ref i2c) = self;
        i2c.conf_ack(true); /* enable ack */
        i2c.start(true, true); /* start condition (for writing addr) */
        i2c.send_addr(AT24C_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(((start >> 8) & 0x0f) as u8, true); /* first word addr */
        i2c.send((start & 0xff) as u8, true);        /* second word addr */
        i2c.start(true, true); /* restart to read */
        i2c.send_addr(AT24C_ADDR, TransDir::RECEIVER, true);
        for i in 0..(size - 1) {
            buf[i] = i2c.recv(true);
        }
        i2c.conf_ack(false); /* nack */
        buf[size - 1] = i2c.recv(true);
        i2c.stop(true);
    }

    pub fn page_write(&self, start: u16, size: usize, buf: &[u8]) {
        let &AT24C(ref i2c) = self;
        i2c.conf_ack(true); /* enable ack */
        i2c.start(true, true); /* start condition (for writing addr) */
        i2c.send_addr(AT24C_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(((start >> 8) & 0x0f) as u8, true); /* first word addr */
        i2c.send((start & 0xff) as u8, true);        /* second word addr */
        for i in 0..size {
            i2c.send(buf[i], true);
        }
        i2c.stop(true);
    }
}

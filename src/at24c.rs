use i2c::{I2C, TransDir};

const AT24C_ADDR: u8 = 0b1010111; /* suppose A0, A1, A2 = 1 */

pub struct AT24C<'a>(&'a I2C<'a>);

impl<'a> AT24C<'a> {
    pub fn new(i2c: &'a I2C<'a>) -> AT24C<'a> {
        AT24C(i2c)
    }

    pub fn read(&self, start: u16, size: usize, buf: &mut [u8]) {
        let &AT24C(ref i2c) = self;
        i2c.conf_ack(true); /* enable ack */
        i2c.start(true, true); /* start condition (for writing addr) */
        while !i2c.send_addr(AT24C_ADDR, TransDir::TRANSMITTER, true) {
            i2c.start(true, true);
        }
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
        while !i2c.send_addr(AT24C_ADDR, TransDir::TRANSMITTER, true) {
            i2c.start(true, true);
        }
        i2c.send(((start >> 8) & 0x0f) as u8, true); /* first word addr */
        i2c.send((start & 0xff) as u8, true);        /* second word addr */
        for i in 0..size {
            i2c.send(buf[i], true);
        }
        i2c.stop(true);
    }

    pub fn write(&self, start: u16, size: usize, buf: &[u8]) {
        let end = start + size as u16 - 1;
        let pg_s_off = start & 0x1f;
        let pg_e_off = end & 0x1f;
        let pg_s = start >> 5;
        let pg_e = end >> 5;
        if pg_s == pg_e {
            self.page_write(start, size, buf);
        } else {
            let mut buf = buf;
            self.page_write(start, (0x20 - pg_s_off) as usize, buf);
            buf = &buf[0x20 - pg_s_off as usize..];
            for i in (pg_s + 1)..pg_e {
                self.page_write(i << 5, 0x20, buf);
                buf = &buf[0x20..];
            }
            self.page_write(pg_e << 5, (pg_e_off + 1) as usize, buf);
        }
    }
}

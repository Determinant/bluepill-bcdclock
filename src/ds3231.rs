extern crate stm32f103xx;
use ::i2c::{I2C, TransDir};

const DS3231_ADDR: u8 = 0b1101000;
const DS3231_REG_SEC: u8 = 0x00;
const DS3231_REG_CTL: u8 = 0x0e;

pub struct DS3231<'a> {
    i2c: I2C<'a>
}

pub struct Date {
    pub second: u8,
    pub minute: u8,
    pub hour: u8,
    pub day: u8,
    pub date: u8,
    pub month: u8,
    pub year: u8,
    pub am: bool,
    pub am_enable: bool
}

impl<'a> DS3231<'a> {
    pub fn new(i2c_reg: &'a stm32f103xx::i2c1::RegisterBlock) -> DS3231<'a> {
        DS3231{i2c: I2C::new(i2c_reg)}
    }

    fn bcd2dec(bcd: u8) -> u8 {
        (bcd >> 4) * 10 + (bcd & 0x0f)
    }

    fn dec2bcd(dec: u8) -> u8 {
        ((dec / 10) << 4) | (dec % 10)
    }

    pub fn init(&self) {
        let i2c = &self.i2c;
        i2c.init();
        i2c.start(true, true);
        i2c.send_addr(DS3231_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(DS3231_REG_CTL, true);
        i2c.send(0x00, true);
        i2c.send(0x00, true);
        i2c.stop(true);
    }

    pub fn read_fulldate(&self) -> Date {
        let mut buf: [u8; 7] = [0; 7];
        let i2c = &self.i2c;
        i2c.conf_ack(true); /* enable ack */
        i2c.start(true, true); /* start condition (for writing reg addr) */
        i2c.send_addr(DS3231_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(DS3231_REG_SEC, true);
        /* restart condition (for reading val from the reg addr) */
        i2c.start(true, true);
        i2c.send_addr(DS3231_ADDR, TransDir::RECEIVER, true);
        for i in 0..6 {
            buf[i] = i2c.recv(true);
        }
        i2c.conf_ack(false); /* disable ack (send nack) */
        buf[6] = i2c.recv(true);
        i2c.stop(true);
        let am_enable = (buf[2] >> 6) & 1 == 1;
        let hour = if am_enable {
            (buf[2] & 0x0f) + ((buf[2] >> 4) & 1) * 10
        } else {
            DS3231::bcd2dec(buf[2])
        };
        let am = if am_enable {(buf[2] >> 5) & 1 == 0} else {hour < 12};
        Date{second: DS3231::bcd2dec(buf[0]),
             minute: DS3231::bcd2dec(buf[1]),
             hour: hour,
             day: DS3231::bcd2dec(buf[3]),
             date: DS3231::bcd2dec(buf[4]),
             month: DS3231::bcd2dec(buf[5]),
             year: DS3231::bcd2dec(buf[6]),
             am: am,
             am_enable: am_enable}
    }

    pub fn write_fulldate(&self, date: &Date) {
        let i2c = &self.i2c;
        let hour = if date.am_enable {
            (1 << 6) | ((if date.am {0} else {1}) << 5) |
            ((date.hour % 10) << 4) | (date.hour & 0x0f)
        } else {
            DS3231::dec2bcd(date.hour)
        };
        let buf: [u8; 7] = [DS3231::dec2bcd(date.second),
                            DS3231::dec2bcd(date.minute),
                            hour,
                            DS3231::dec2bcd(date.day),
                            DS3231::dec2bcd(date.date),
                            DS3231::dec2bcd(date.month),
                            DS3231::dec2bcd(date.year)];
        i2c.conf_ack(true);
        i2c.start(true, true); /* start condition for writing */
        i2c.send_addr(DS3231_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(DS3231_REG_SEC, true);
        for i in 0..7 {
            i2c.send(buf[i], true);
        }
        i2c.stop(true);
    }
}

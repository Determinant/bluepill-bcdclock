use i2c::{I2C, TransDir, DutyType};

const DS3231_ADDR: u8 = 0b1101000;
const DS3231_REG_SEC: u8 = 0x00;
const DS3231_REG_CTL: u8 = 0x0e;
const DS3231_REG_TEMP: u8 = 0x11;

pub struct DS3231<'a>(pub &'a I2C<'a>);

pub struct Date {
    pub second: u8,
    pub minute: u8,
    pub hour: u8,
    pub day: u8,
    pub date: u8,
    pub month: u8,
    pub year: u8,
    pub am: bool,
    pub am_enabled: bool
}

pub struct Temp {
    pub cels: i8,
    pub quarter: u8
}

impl<'a> DS3231<'a> {
    fn bcd2dec(bcd: u8) -> u8 {
        (bcd >> 4) * 10 + (bcd & 0x0f)
    }

    fn dec2bcd(dec: u8) -> u8 {
        ((dec / 10) << 4) | (dec % 10)
    }

    fn read_register(&self, start: u8, size: usize, buf: &mut [u8]){
        let &DS3231(ref i2c) = self;
        i2c.conf_ack(true); /* enable ack */
        i2c.start(true, true); /* start condition (for writing reg addr) */
        i2c.send_addr(DS3231_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(start, true);
        /* restart condition (for reading val from the reg addr) */
        i2c.start(true, true);
        i2c.send_addr(DS3231_ADDR, TransDir::RECEIVER, true);
        for i in 0..(size - 1) {
            buf[i] = i2c.recv(true);
        }
        i2c.conf_ack(false); /* disable ack (send nack) */
        buf[size - 1] = i2c.recv(true);
        i2c.stop(true);
    }

    fn write_register(&self, start: u8, size: usize, buf: &[u8]) {
        let &DS3231(ref i2c) = self;
        i2c.conf_ack(true);
        i2c.start(true, true); /* start condition for writing */
        i2c.send_addr(DS3231_ADDR, TransDir::TRANSMITTER, true);
        i2c.send(start, true);
        for i in 0..size {
            i2c.send(buf[i], true);
        }
        i2c.stop(true);
    }

    pub fn read_fulldate(&self) -> Date {
        let mut buf: [u8; 7] = [0; 7];
        self.read_register(DS3231_REG_SEC, 7, &mut buf);
        let am_enabled = (buf[2] >> 6) & 1 == 1;
        let hour = if am_enabled {
            (buf[2] & 0x0f) + ((buf[2] >> 4) & 1) * 10
        } else {
            DS3231::bcd2dec(buf[2])
        };
        let am = if am_enabled {(buf[2] >> 5) & 1 == 0} else {hour < 12};
        Date{second: DS3231::bcd2dec(buf[0]),
             minute: DS3231::bcd2dec(buf[1]),
             hour,
             day: DS3231::bcd2dec(buf[3]),
             date: DS3231::bcd2dec(buf[4]),
             month: DS3231::bcd2dec(buf[5]),
             year: DS3231::bcd2dec(buf[6]),
             am,
             am_enabled}
    }

    pub fn write_fulldate(&self, date: &Date) {
        let hour = if date.am_enabled {
            (1 << 6) | ((if date.am {0} else {1}) << 5) |
            ((date.hour / 10) << 4) | (date.hour % 10)
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
        self.write_register(DS3231_REG_SEC, 7, &buf);
    }

    pub fn write_control(&self) {
        let buf: [u8; 2] = [0; 2];
        self.write_register(DS3231_REG_CTL, 2, &buf);
    }

    pub fn read_temperature(&self) -> Temp {
        let mut buf: [u8; 2] = [0; 2];
        self.read_register(DS3231_REG_TEMP, 2, &mut buf);
        Temp{cels: buf[0] as i8, quarter: buf[1] >> 6}
    }
}

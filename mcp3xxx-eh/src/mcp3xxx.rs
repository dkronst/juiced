// We'll start with the "mcp3xxx.rs"

use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::Write;
use std::collections::HashMap;

pub trait MCP3xxx<SPI, CS> {
    fn new(spi: SPI, cs: CS) -> Self where Self: Sized;
    fn reference_voltage(&self) -> f32;
    fn read(&mut self, pin: u8, is_differential: bool) -> u16;
    fn diff_pins(&self) -> HashMap<(u8, u8), u8>;
}

pub struct SPIDevice<SPI, CS> {
    spi: SPI,
    cs: CS,
}

impl<SPI, CS> SPIDevice<SPI, CS>
where
    SPI: Write<u8>,
    CS: OutputPin,
{
    pub fn new(spi: SPI, cs: CS) -> Self {
        SPIDevice { spi, cs }
    }

    pub fn reference_voltage(&self) -> f32 {
        3.3
    }

    pub fn read(&mut self, pin: u8, is_differential: bool) -> u16 {
        let out_buf = [0x40 | ((!is_differential) as u8) << 5 | pin << 4, 0x00];
        let mut in_buf = [0x00, 0x00];
        self.cs.set_low().ok();
        let _ = self.spi.write(&out_buf);
        let _ = self.spi.write(&mut in_buf);
        self.cs.set_high().ok();
        ((in_buf[0] & 0x03) as u16) << 8 | in_buf[1] as u16
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use embedded_hal_mock::{spi::Transaction as SPITransaction, spi::Mock as MockSPI, pin::Mock as MockPin};

    #[test]
    fn it_reads_value() {
        let expectations = [
            SPITransaction::write(vec![0x80, 0x00]),
            SPITransaction::transfer(vec![0x00, 0x00], vec![0x03, 0xFF]),
        ];
        let mock_spi = MockSPI::new(&expectations);
        let mock_pin = MockPin::new(&[Ok(())]);
        let mut device = SPIDevice::new(mock_spi, mock_pin);
        assert_eq!(device.read(0, false), 1023);
    }
}

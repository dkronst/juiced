// Finally, "mcp3002.rs"

use super::mcp3xxx::{MCP3xxx, SPIDevice};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::Write;

use std::collections::HashMap;

pub struct MCP3002<SPI, CS> {
    spi: SPIDevice<SPI, CS>,
}

impl<SPI, CS> MCP3xxx<SPI, CS> for MCP3002<SPI, CS>
where
    SPI: Write<u8>,
    CS: OutputPin,
{
    fn new(spi: SPI, cs: CS) -> Self {
        MCP3002 {
            spi: SPIDevice::new(spi, cs),
        }
    }

    fn reference_voltage(&self) -> f32 {
        self.spi.reference_voltage()
    }

    fn read(&mut self, pin: u8, is_differential: bool) -> u16 {
        self.spi.read(pin, is_differential)
    }

    fn diff_pins(&self) -> HashMap<(u8, u8), u8> {
        let mut map = HashMap::new();
        map.insert((0, 1), 0);
        map.insert((1, 0), 1);
        map
    }
}

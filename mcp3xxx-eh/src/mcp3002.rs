// Finally, "mcp3002.rs"

use super::mcp3xxx::{MCP3xxx, SPIDevice};
use embedded_hal::spi::{Mode, MODE_0};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::Write;

pub struct MCP3002<SPI, CS> {
    mcp: SPIDevice<SPI, CS>,
}

impl<SPI, CS> MCP3xxx for MCP3002<SPI, CS>
where
    SPI: Write<u8>,
    CS: OutputPin,
{
    fn reference_voltage(&self) -> f32 {
        self.mcp.reference_voltage()
    }

    fn read(&mut self, pin: u8, is_differential: bool) -> u16 {
        self.mcp.read(pin, is_differential)
    }
}

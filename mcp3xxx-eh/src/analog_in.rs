// Next, we'll define "analog_in.rs"

use super::mcp3xxx::{MCP3xxx};
use embedded_hal::digital::v2::OutputPin;
use embedded_hal::blocking::spi::Write;

pub struct AnalogIn<'a, SPI, CS> {
    mcp: &'a dyn MCP3xxx<SPI, CS>,
    pin_setting: u8,
    is_differential: bool,
}

impl<'a, SPI, CS> AnalogIn<'a, SPI, CS>
where
    SPI: Write<u8>,
    CS: OutputPin,
{
    pub fn new(mcp: &dyn MCP3xxx<SPI, CS>, positive_pin: u8, negative_pin: Option<u8>) -> Self {
        let is_differential = negative_pin.is_some();

        if is_differential {
            mcp.diff_pins().contains_key(&(positive_pin, negative_pin.unwrap()));
        }

        let pin_setting = if is_differential {
            match negative_pin {
                Some(np) => {
                    let x = mcp.diff_pins();
                    x.get(&(positive_pin, np))
                }
                None => panic!("Invalid differential pin mapping"),
            }
        } else {
            Some(&positive_pin)
        };
        // unwrap() is safe here because we know that the pin setting is valid
        let pin_setting = *pin_setting.unwrap();
        AnalogIn { mcp, pin_setting, is_differential }
    }

    pub fn value(&mut self) -> u16 {
        self.mcp.read(self.pin_setting, self.is_differential) << 6
    }

    pub fn voltage(&mut self) -> f32 {
        (self.value() as f32 * self.mcp.reference_voltage()) / 65535.0
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
        let mut analog_in = AnalogIn::new(device, 0, None);
        assert_eq!(analog_in.value(), 65472);
    }

    #[test]
    fn it_reads_voltage() {
        let expectations = [
            SPITransaction::write(vec![0x80, 0x00]),
            SPITransaction::transfer(vec![0x00, 0x00], vec![0x03, 0xFF]),
        ];
        let mock_spi = MockSPI::new(&expectations);
        let mock_pin = MockPin::new(&[Ok(())]);
        let mut device = SPIDevice::new(mock_spi, mock_pin);
        let mut analog_in = AnalogIn::new(device, 0, None);
        assert_eq!(analog_in.voltage(), 3.3);
    }
}

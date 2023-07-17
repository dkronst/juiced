use rppal::spi::{Bus, Mode, SlaveSelect, Spi, Error as LibError};

// This file defines a private (to this crate) struct called Adc. It has a
// public method called new() which returns a Result<Adc, AdcError>. The
// The ADC uses a mcp3008 chip which is connected to the Raspberry Pi via SPI.
// The mcp3008 is connected to the Raspberry Pi with the first SPI bus (SPI0)

// 3 Channels are connected to the SPI:
// 1. Pilot voltage
// 2. Current sense
// 3. AC Voltage

// Define the struct:
pub struct Adc {
    mcp: Spi,
}

// Define the error type:
#[derive(Debug)]
pub enum AdcError {
    SpiError(std::io::Error),
    LibError(LibError),
}

impl From<LibError> for AdcError {
    fn from(error: LibError) -> Self {
        AdcError::LibError(error)
    }
}

impl From<std::io::Error> for AdcError {
    fn from(error: std::io::Error) -> Self {
        AdcError::SpiError(error)
    }
}

// Define the trait:
pub trait Mcp3004 {
    fn single_ended_read(&mut self, channel: u8) -> Result<u16, LibError>;
}

impl Mcp3004 for Spi {
    fn single_ended_read(&mut self, channel: u8) -> Result<u16, LibError> {
        let mut read_buffer = [0u8; 3];
        let write_buffer = [0b0000_0001, (0b1000_0000 | (channel << 4)) as u8, 0];

        println!("Write buffer: {:?}", write_buffer);
        self.transfer(&mut read_buffer, &write_buffer)?;

        let value = ((read_buffer[1] as u16 & 0x03) << 8) | read_buffer[2] as u16;

        println!("Channel: {}, Value: {}", channel, value);

        Ok(value)
    }
}

// Implement the Adc struct:
impl Adc {
    const PILOT_VOLTAGE_CHANNEL: u8 = 0;
    const CURRENT_SENSE_CHANNEL: u8 = 1;
    const AC_VOLTAGE_CHANNEL:    u8 = 2;
    const SPI_FREQUENCY:        u32 = 1_000_000;
    const SPI_MODE:            Mode = Mode::Mode0;
    const SPI_BUS:              Bus = Bus::Spi0;
    const SPI_SLAVE_SELECT: SlaveSelect = SlaveSelect::Ss0;
    const REFERENCE_VOLTAGE:    f32 = 5.0;

    pub fn new() -> Result<Self, AdcError> {
        let spi = Spi::new(Self::SPI_BUS, Self::SPI_SLAVE_SELECT, Self::SPI_FREQUENCY, Self::SPI_MODE)?;

        Ok(Self {
            mcp: spi,
        })
    }

    fn to_volts(reading: u16) -> f32 {
        let voltage = (reading as f32) * Self::REFERENCE_VOLTAGE / 1024.0;
        voltage
    }

    fn to_amps(reading: u16) -> f32 {
        let voltage = (reading as f32) * Self::REFERENCE_VOLTAGE / 1024.0;
        let amps = (voltage - 1.65) / 0.066;
        amps
    }

    pub fn read_pilot_voltage(&mut self) -> Result<f32, AdcError> {
        let reading = self.mcp.single_ended_read(Self::PILOT_VOLTAGE_CHANNEL)?;
        let voltage = Self::to_volts(reading);
        Ok(voltage)
    }

    pub fn read_current_sense(&mut self) -> Result<f32, AdcError> {
        let reading = self.mcp.single_ended_read(Self::CURRENT_SENSE_CHANNEL)?;
        let curr = Self::to_amps(reading);
        Ok(curr)
    }

}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_to_volts() {
        let reading = 512;
        let volts = Adc::to_volts(reading);
        assert_eq!(volts, 1.65);
    }

    #[test]
    fn test_to_amps() {
        let reading = 512;
        let amps = Adc::to_amps(reading);
        assert_eq!(amps, 0.0);
    }

    #[test]
    fn test_read_pilot_voltage() -> Result<(), AdcError> {
        let mut adc = Adc::new()?;
        let voltage = adc.read_pilot_voltage()?;
        assert!(voltage >= 0.1 && voltage <= 5.0);
        Ok(())
    }

    #[test]
    fn test_read_current_sense() -> Result<(), AdcError> {
        let mut adc = Adc::new()?;
        let current = adc.read_current_sense()?;
        assert!(current >= -50.0 && current <= 50.0);
        Ok(())
    }

    #[test]
    fn test_ac_voltage() {
        let mut adc = Adc::new().unwrap();
        let reading = adc.mcp.single_ended_read(Adc::AC_VOLTAGE_CHANNEL).unwrap();
        let voltage = Adc::to_volts(reading);
        println!("AC Voltage: {}", voltage);
    }
}
use crate::mcp::lib::{Channel, Mcp3004, Error as LibError};
use rppal::spi::{Bus, Mode, SlaveSelect, Spi};

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
    mcp: Mcp3004
}

// Define the error type:
#[derive(Debug)]
pub enum AdcError {
    SpiError(std::io::Error),
    LibError(LibError),
}

// Implement From for the standard error type
impl From<std::io::Error> for AdcError {
    fn from(error: std::io::Error) -> Self {
        Self::SpiError(error)
    }
}

impl From<LibError> for AdcError {
    fn from(error: LibError) -> Self {
        AdcError::LibError(error)
    }
}


// Implement the Adc struct:
impl Adc {
    pub fn new() -> Result<Self, AdcError> {
        let spi = Spi::new(Bus::Spi0, SlaveSelect::Ss0, 1_000_000, Mode::Mode0).unwrap();
        let mcp3004 = Mcp3004::new(spi).unwrap();

        Ok(Self {
            mcp: mcp3004,
        })
    }

    fn to_volts(reading: u16) -> f32 {
        let voltage = (reading as f32) * 3.3 / 1024.0;
        voltage
    }

    fn to_amps(reading: u16) -> f32 {
        let voltage = (reading as f32) * 3.3 / 1024.0;
        let amps = (voltage - 1.65) / 0.066;
        amps
    }

    pub fn read_pilot_voltage(&mut self) -> Result<f32, AdcError> {
        let reading = self.mcp.single_ended_read(Channel(0))?;
        let voltage = Self::to_volts(reading.value());
        Ok(voltage)
    }

    pub fn read_current_sense(&mut self) -> Result<f32, AdcError> {
        let reading = self.mcp.single_ended_read(Channel(1))?;
        let curr = Self::to_amps(reading.value());
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
        assert!(voltage >= 0.0 && voltage <= 3.3);
        Ok(())
    }

    #[test]
    fn test_read_current_sense() -> Result<(), AdcError> {
        let mut adc = Adc::new()?;
        let current = adc.read_current_sense()?;
        assert!(current >= -50.0 && current <= 50.0);
        Ok(())
    }
}
use std::time::{Duration, Instant};

use rppal::spi::{Bus, Mode, SlaveSelect, Spi, Error as LibError};
use log::debug;

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
    fn single_ended_read(&self, channel: u8) -> Result<u16, LibError>;
}

impl Mcp3004 for Spi {
    fn single_ended_read(&self, channel: u8) -> Result<u16, LibError> {
        let mut read_buffer = [0u8; 3];
        let write_buffer = [0b0000_0001, 0b1100_0000 | channel << 4, 0b0000_0000];

        self.transfer(&mut read_buffer, &write_buffer)?;

        let value = ((read_buffer[1] as u16 & 0x03) << 8) | read_buffer[2] as u16;

        Ok(value)
    }
}

// Implement the Adc struct:
impl Adc {
    const PILOT_VOLTAGE_CHANNEL: u8 = 0;
    const CURRENT_SENSE_CHANNEL: u8 = 1;
    const AC_VOLTAGE_CHANNEL:    u8 = 2;
    const CS_SCALE_FACTOR:      f32 = (16./(6.75))/0.066;    // => 16A -> x = 6.75 y(6.75) = 16A   y(0) = 0, x = t/0.066 => y(t) = t/0.066 * 6.75
    const SPI_FREQUENCY:        u32 = 300_000;  // Tested empirically, 300kHz is the fastest that works linearly.
    const SPI_MODE:            Mode = Mode::Mode0;
    const SPI_BUS:              Bus = Bus::Spi0;
    const SPI_SLAVE_SELECT: SlaveSelect = SlaveSelect::Ss0;
    const REFERENCE_VOLTAGE:    f32 = 5.0;
    /// Mains period at 50 Hz. RMS integration runs for exactly this long,
    /// so a phase boundary inside the window can't bias the result. At the
    /// 300 kHz SPI clock with 24-bit reads we get ≈ 250 samples per window.
    pub const RMS_WINDOW: Duration = Duration::from_millis(20);
    /// Reported in `Waveform` so the operator can sanity-check the effective
    /// sample rate from the recorded data.
    pub const SPI_HZ: u32 = Self::SPI_FREQUENCY;

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
        let amps = (voltage - Self::REFERENCE_VOLTAGE/2.) * Self::CS_SCALE_FACTOR;
        amps
    }

    pub fn read_pilot_voltage(&mut self) -> Result<f32, AdcError> {
        let reading = self.mcp.single_ended_read(Self::PILOT_VOLTAGE_CHANNEL)?;
        let voltage = Self::to_volts(reading);
        Ok(voltage)
    }

    #[inline]
    fn from_vdiv_to_pilot(voltage: f32) -> f32 {      
        // 0.9V means -12V, 4.55V means +12V. Between those values, the reading should be linear.
        // Linear equation: y-y_0 = M(x-x_0)
        // M = (V_p-V_0p)/(V_d-V_0d) = (12-(-12))/(4.55-0.9) = 24/3.65
        // V_0p = -12, V_0d = 0.9
        // V_p = 24/3.65*V_d - 24/3.65*0.9 - 12
        let pilot_voltage = 24./3.65*voltage - 24./3.65*0.9 - 12.;
        debug!("Vdiv: {}, Pilot: {}", voltage, pilot_voltage);
        pilot_voltage
    }

    #[inline]
    pub fn peak_to_peak_pilot(&self) -> Result<(f32, f32), AdcError> {
        // TODO: fix so that the voltage is corrected for the voltage divider
        let (min, max) = self.peak_to_peak(Self::PILOT_VOLTAGE_CHANNEL, Duration::from_millis(50))?; // 10 cycles?
        Ok((Self::from_vdiv_to_pilot(min), Self::from_vdiv_to_pilot(max)))
    }


    /// Integrate the squared current-sense reading for exactly one 50 Hz
    /// mains cycle (`RMS_WINDOW`, 20 ms). At the 300 kHz SPI clock this
    /// yields ≈ 250 samples evenly distributed across the cycle, so phase
    /// has only a sub-1 % effect on the result. Replaces the previous
    /// "20 samples in a tight loop ≈ 1.7 ms" approach which captured less
    /// than 10 % of a cycle at random phase and caused the choppy display.
    pub fn read_current_sense_rms(&self) -> Result<f32, AdcError> {
        let deadline = Instant::now() + Self::RMS_WINDOW;
        // Accumulate in f64 because we may add ~250 squares of ±~20-ish; f32
        // precision is fine here, but f64 keeps the math obviously safe.
        let mut sum_sq: f64 = 0.0;
        let mut count: u32 = 0;
        while Instant::now() < deadline {
            let amps = self.read_current_sense_one_sample()?;
            sum_sq += (amps as f64) * (amps as f64);
            count += 1;
        }
        if count == 0 {
            // Shouldn't happen — clock didn't advance? — but defend anyway.
            return Ok(0.0);
        }
        debug!("Current sense RMS: {} samples over ~{} ms", count, Self::RMS_WINDOW.as_millis());
        Ok((sum_sq / count as f64).sqrt() as f32)
    }

    /// Read a single current-sense sample. Public so the synchronous-
    /// undersampling capture in `juicelib::evse` can schedule its own
    /// per-sample timing without going through the burst-read primitives.
    #[inline]
    pub fn read_current_sense(&self) -> Result<f32, AdcError> {
        let reading = self.mcp.single_ended_read(Self::CURRENT_SENSE_CHANNEL)?;
        let curr = Self::to_amps(reading);
        Ok(curr)
    }

    #[inline]
    #[allow(dead_code)]
    fn read_current_sense_one_sample(&self) -> Result<f32, AdcError> {
        self.read_current_sense()
    }

    /// Sample the current-sense channel as fast as the SPI allows for
    /// exactly `window` time, returning every raw sample with a monotonic
    /// `(seconds_since_window_start, amps)` timestamp.
    ///
    /// Used by the diagnostic waveform capture path in `start_adc_thread`
    /// to record what the analog front end actually looks like (full-wave,
    /// half-wave, biased, clipped, …). Not used by the normal RMS path.
    pub fn capture_current_waveform_window(
        &self,
        window: Duration,
    ) -> Result<Vec<(f32, f32)>, AdcError> {
        let start = Instant::now();
        let deadline = start + window;
        // Pre-allocate for ~12.5 kHz over `window`; rough cap, will grow if
        // SPI is faster than expected.
        let estimated = ((window.as_secs_f64() * 13_000.0) as usize).max(64);
        let mut samples = Vec::with_capacity(estimated);
        while Instant::now() < deadline {
            let t = start.elapsed().as_secs_f32();
            let amps = self.read_current_sense_one_sample()?;
            samples.push((t, amps));
        }
        Ok(samples)
    }

    pub fn peak_to_peak(&self, channel: u8, duration: Duration) -> Result<(f32, f32), AdcError> {
        // This can read around 290*50 = 14500 samples per second which should be enough
        let mut min = 1024;
        let mut max = 0;
        let start = std::time::Instant::now();

        while start.elapsed() < duration {  
            let reading = self.mcp.single_ended_read(channel)?;
            if reading < min {
                min = reading;
            }
            if reading > max {
                max = reading;
            }
        }
        let min_volts = Self::to_volts(min);
        let max_volts = Self::to_volts(max);
        Ok((min_volts, max_volts))
    }

    /// Returns the peak voltage of the AC voltage.
    pub fn peak_mains_voltage(&self) -> Result<f32, AdcError> {
        let (min, max) = self.peak_to_peak(Self::AC_VOLTAGE_CHANNEL, Duration::from_millis(2*1000/50))?;
        let mains_peak = if min.abs() > max.abs() {
            min
        } else {
            max
        };        
        Ok(mains_peak)
    }
}

#[cfg(test)]
mod tests {
    use log::info;

    use super::*;

    #[test]
    fn test_to_volts() {
        let reading = 512;
        let volts = Adc::to_volts(reading);
        assert_eq!(volts, Adc::REFERENCE_VOLTAGE/2.);
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
        let adc = Adc::new()?;
        let current = adc.read_current_sense_rms()?;
        assert!(current >= -50.0 && current <= 50.0);
        Ok(())
    }

    #[test]
    fn test_ac_voltage() {
        let adc = Adc::new().unwrap();
        let reading = adc.mcp.single_ended_read(Adc::AC_VOLTAGE_CHANNEL).unwrap();
        let voltage = Adc::to_volts(reading);

        let p2pv = adc.peak_to_peak(Adc::AC_VOLTAGE_CHANNEL, Duration::from_millis(1000/50));
        assert!(p2pv.is_ok());
        let (min, max) = p2pv.unwrap();

        info!("Min: {}, Max: {}", min, max);
        info!("AC Voltage: {}", voltage);
    }

    #[test]
    fn test_vdiv_pilot() {
        println!("0.9V means -12V, 4.55V means +12V. Between those values, the reading should be linear.");
        println!("Linear equation: y-y_0 = M(x-x_0)");
        for i in 0..10 {
            let voltage = 0.9 + i as f32 * (4.55-0.9)/10.;
            let pilot_voltage = Adc::from_vdiv_to_pilot(voltage);
            println!("Vdiv: {}, Pilot: {}", voltage, pilot_voltage);
        }
        assert_eq!(Adc::from_vdiv_to_pilot(4.55), 12.0);
        assert_eq!(Adc::from_vdiv_to_pilot(0.9), -12.0);
    }
}
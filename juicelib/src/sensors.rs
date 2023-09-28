use std::{time::Instant, fmt::{Display, Formatter, self}, borrow::Borrow};

use statrs::statistics::Statistics;

/// Implementation of some of the measurements that the juicelib provides.
/// 


#[derive(Debug)]
pub struct SensorsState {
    current_sensor: Vec<SensorReading>,
    mains_peak: Vec<SensorReading>,
    pub last_update: Instant,
}

impl Display for SensorsState {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "Current Sensor: {:?}, Mains Peak: {:?}", self.get_cs_avg(), self.get_mains_avg())
    }
}

#[derive(Debug)]
struct SensorReading {
    reading: f64,
    timestamp: Instant,
}

impl SensorReading {
    pub fn new(reading: f64) -> Self {
        SensorReading {
            reading,
            timestamp: Instant::now(),
        }
    }
}

impl Borrow<f64> for &SensorReading {
    fn borrow(&self) -> &f64 {
        &self.reading
    }
}

impl SensorsState {
    const MAX_READINGS: usize = 100;
    pub fn new() -> Self {
        SensorsState {
            current_sensor: vec![],
            mains_peak: vec![],
            last_update: Instant::now(),
        }
    }

    pub fn get_cs_avg(&self) -> f32 {
        self.current_sensor.iter().geometric_mean() as f32
    }

    pub fn get_mains_avg(&self) -> f32 {
        self.mains_peak.iter().geometric_mean() as f32
    }

    pub fn add_cs_reading(&mut self, reading: f64) {
        self.current_sensor.push(SensorReading::new(reading));
        if self.current_sensor.len() > Self::MAX_READINGS {
            self.current_sensor.remove(0);
        }
        self.last_update = Instant::now();
    }

    pub fn add_mains_peak_reading(&mut self, reading: f64) {
        self.mains_peak.push(SensorReading::new(reading));
        if self.mains_peak.len() > Self::MAX_READINGS {
            self.mains_peak.remove(0);
        }
        self.last_update = Instant::now();
    }
}

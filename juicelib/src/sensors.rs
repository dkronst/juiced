use std::time::Instant;

/// Implementation of some of the measurements that the juicelib provides.
/// 


#[derive(Debug)]
pub struct SensorsState {
    pub current_sensor: f32,
    pub mains_peak: f32,
    pub last_update: Instant,
}

impl SensorsState {
    pub fn new() -> Self {
        SensorsState {
            current_sensor: 0.0,
            mains_peak: 0.0,
            last_update: Instant::now(),
        }
    }
}

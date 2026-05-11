//! Local-only demo. Drives a `StatusBroker` through a realistic plug-in →
//! charge → unplug cycle while `juiced_web::serve` exposes the dashboard at
//! http://localhost:8080. Useful when there's no Pi attached.
//!
//! Run: `cargo run -p juiced-web --example mock`

use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use juiced_status::{
    FsmState, StatusBroker, SupervisorPhase, Waveform, WaveformSample,
};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let broker = Arc::new(StatusBroker::new());
    let reader = broker.reader();

    thread::Builder::new()
        .name("juiced-web".to_string())
        .spawn(move || juiced_web::serve(reader, 8080))
        .expect("spawn web thread");

    // Demo continuous capture: publish a fabricated waveform every ~4 s,
    // mirroring the real ADC thread's pacing.
    let broker_for_capture = Arc::clone(&broker);
    thread::Builder::new()
        .name("mock-capture".to_string())
        .spawn(move || demo_capture_worker(broker_for_capture))
        .expect("spawn capture worker");

    println!();
    println!("  ┌───────────────────────────────────────────────┐");
    println!("  │  open http://localhost:8080/ in your browser  │");
    println!("  └───────────────────────────────────────────────┘");
    println!();

    drive_scenario(broker);
}

fn drive_scenario(broker: Arc<StatusBroker>) -> ! {
    loop {
        log::info!("scenario: boot");
        broker.set_supervisor(SupervisorPhase::Initializing);
        thread::sleep(Duration::from_secs(2));

        broker.set_supervisor(SupervisorPhase::Running);
        broker.set_fsm_state(FsmState::SelfTest);
        broker.update_pilot_voltage(-12.0, -12.0);
        thread::sleep(Duration::from_secs(3));

        log::info!("scenario: standby, waiting for vehicle");
        broker.set_fsm_state(FsmState::Standby);
        broker.update_pilot_voltage(-12.0, 12.0);
        broker.update_current_amps(0.0);
        broker.update_mains_voltage(240.0);
        broker.set_contactor_on(false);
        thread::sleep(Duration::from_secs(6));

        log::info!("scenario: vehicle plugged in");
        broker.set_fsm_state(FsmState::VehicleDetected);
        broker.update_pilot_voltage(-12.0, 9.0);
        thread::sleep(Duration::from_secs(4));

        log::info!("scenario: vehicle requests charge");
        broker.set_fsm_state(FsmState::StartCharging);
        broker.update_pilot_voltage(-12.0, 6.0);
        thread::sleep(Duration::from_secs(3));

        log::info!("scenario: charging");
        broker.set_fsm_state(FsmState::Charging);
        broker.set_contactor_on(true);
        // Ramp current up over ~5 s, then hold steady.
        for tick in 0..40 {
            let target = 16.0_f32;
            let amps = if tick < 10 {
                (tick as f32 + 1.0) / 10.0 * target
            } else {
                target + ((tick as f32 * 0.7).sin() * 0.4)
            };
            let mains = 240.0 + ((tick as f32 * 1.1).sin() * 2.0);
            broker.update_current_amps(amps);
            broker.update_mains_voltage(mains);
            thread::sleep(Duration::from_millis(500));
        }

        log::info!("scenario: vehicle signals done");
        broker.set_fsm_state(FsmState::StopCharging);
        broker.update_pilot_voltage(-12.0, 9.0);
        broker.update_current_amps(0.0);
        broker.set_contactor_on(false);
        thread::sleep(Duration::from_secs(3));

        broker.set_fsm_state(FsmState::NoPower);
        thread::sleep(Duration::from_secs(3));

        log::info!("scenario: vehicle unplugged, back to standby");
        broker.set_fsm_state(FsmState::Standby);
        broker.update_pilot_voltage(-12.0, 12.0);
        thread::sleep(Duration::from_secs(5));

        log::info!("scenario: simulated transient fault → 8 s restart cycle");
        broker.set_supervisor(SupervisorPhase::RestartingIn {
            seconds_remaining: 8,
            reason: "demo: simulated SPI read failure".to_string(),
        });
        broker.set_fsm_state(FsmState::Unknown);
        thread::sleep(Duration::from_secs(8));
    }
}

fn demo_capture_worker(broker: Arc<StatusBroker>) -> ! {
    loop {
        // Match the real ADC thread's cadence: a capture takes ~4 s.
        thread::sleep(Duration::from_secs(4));
        broker.publish_waveform(fabricate_waveform());
    }
}

/// 200 phase-walked samples of a half-wave-rectified-ish 50 Hz sine, so the
/// dashboard's WaveformCard has something realistic to render off-Pi.
fn fabricate_waveform() -> Waveform {
    let n: u32 = 200;
    let mains_period_s: f32 = 1.0 / 50.0;
    let inter_sample_s: f32 = mains_period_s + mains_period_s / (n as f32);
    let mut samples = Vec::with_capacity(n as usize);
    for i in 0..n {
        let t_s = (i as f32) * inter_sample_s;
        // Phase within one 50 Hz cycle.
        let phase = (t_s / mains_period_s).fract();
        let theta = 2.0 * std::f32::consts::PI * phase;
        let raw = theta.sin();
        // Asymmetric: positive lobe full amplitude, negative lobe attenuated,
        // so the operator can clearly see whether the front end is bipolar.
        let amps = if raw >= 0.0 { 16.0 * raw } else { 2.0 * raw };
        samples.push(WaveformSample { t_s, amps });
    }
    Waveform {
        captured_at_unix_ms: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0),
        samples,
        samples_per_cycle: n,
        cycle_period_ms: (mains_period_s * 1000.0) as u32,
        spi_hz: 300_000,
        zero_crossing_found: true,
    }
}

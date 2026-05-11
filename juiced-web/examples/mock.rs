//! Local-only demo. Drives a `StatusBroker` through a realistic plug-in →
//! charge → unplug cycle while `juiced_web::serve` exposes the dashboard at
//! http://localhost:8080. Useful when there's no Pi attached.
//!
//! Run: `cargo run -p juiced-web --example mock`

use std::sync::Arc;
use std::thread;
use std::time::{Duration, SystemTime, UNIX_EPOCH};

use juiced_status::{
    FsmState, StatusBroker, SupervisorPhase, Waveform, WaveformSample, WaveformTrigger,
};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let broker = Arc::new(StatusBroker::new());
    let trigger = WaveformTrigger::new();
    let reader = broker.reader();
    let trig_for_web = trigger.clone();

    thread::Builder::new()
        .name("juiced-web".to_string())
        .spawn(move || juiced_web::serve(reader, trig_for_web, 8080))
        .expect("spawn web thread");

    // Demo capture worker: when the UI presses Capture, fabricate a
    // realistic 50-cycle rectified sine and publish it.
    let broker_for_capture = Arc::clone(&broker);
    let trig_for_capture = trigger.clone();
    thread::Builder::new()
        .name("mock-capture".to_string())
        .spawn(move || demo_capture_worker(broker_for_capture, trig_for_capture))
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

fn demo_capture_worker(broker: Arc<StatusBroker>, trigger: WaveformTrigger) -> ! {
    loop {
        if trigger.take() {
            log::info!("demo capture: generating fake waveform");
            // Pretend it took 10 s — match the real ADC thread's pacing.
            thread::sleep(Duration::from_secs(10));
            broker.publish_waveform(fabricate_waveform());
            log::info!("demo capture: published");
        }
        thread::sleep(Duration::from_millis(200));
    }
}

/// 50 windows × 20 ms, each ~250 samples of a 50 Hz half-wave rectified
/// "almost-sine" with some jitter, so the UI has something interesting to
/// plot in mock mode.
fn fabricate_waveform() -> Waveform {
    let window_count: u32 = 50;
    let window_ms: u32 = 20;
    let samples_per_window: u32 = 250;
    let mut samples = Vec::with_capacity((window_count * samples_per_window) as usize);
    for w in 0..window_count {
        let window_start_s = (w as f32) * 0.2;
        for s in 0..samples_per_window {
            let t_in_window = (s as f32) / (samples_per_window as f32) * 0.02;
            let theta = 2.0 * std::f32::consts::PI * (t_in_window / 0.02);
            // Half-wave rectified-ish: positive lobe full amplitude, negative
            // lobe a fraction (simulating partial bipolar response).
            let raw = theta.sin();
            let amps = if raw >= 0.0 { 16.0 * raw } else { 2.0 * raw };
            samples.push(WaveformSample {
                t_s: window_start_s + t_in_window,
                amps,
            });
        }
    }
    Waveform {
        captured_at_unix_ms: SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0),
        samples,
        window_count,
        window_ms,
        spi_hz: 300_000,
    }
}

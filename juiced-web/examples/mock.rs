//! Local-only demo. Drives a `StatusBroker` through a realistic plug-in →
//! charge → unplug cycle while `juiced_web::serve` exposes the dashboard at
//! http://localhost:8080. Useful when there's no Pi attached.
//!
//! Run: `cargo run -p juiced-web --example mock`

use std::sync::Arc;
use std::thread;
use std::time::Duration;

use juiced_status::{FsmState, StatusBroker, SupervisorPhase};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let broker = Arc::new(StatusBroker::new());
    let reader = broker.reader();

    thread::Builder::new()
        .name("juiced-web".to_string())
        .spawn(move || juiced_web::serve(reader, 8080))
        .expect("spawn web thread");

    println!();
    println!("  ┌──────────────────────────────────────────────┐");
    println!("  │  open http://localhost:8080/ in your browser  │");
    println!("  └──────────────────────────────────────────────┘");
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

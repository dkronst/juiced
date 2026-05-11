
use juicelib::evse::{start_machine, EVSEHardwareImpl};
use log::error;
use std::process::ExitCode;

fn main() -> ExitCode {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let evse = match EVSEHardwareImpl::new() {
        Ok(evse) => evse,
        Err(e) => {
            error!("Failed to initialise EVSE hardware: {:?}", e);
            return ExitCode::FAILURE;
        }
    };
    if let Err(e) = start_machine(evse) {
        error!("State machine exited with error: {:?}", e);
        return ExitCode::FAILURE;
    }
    ExitCode::SUCCESS
}

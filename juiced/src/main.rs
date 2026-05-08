
use juicelib::evse::{start_machine, EVSEHardwareImpl};

fn main() {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    let evse = EVSEHardwareImpl::new();
    start_machine(evse);
}


use juicelib::evse::{start_machine, EVSEHardwareImpl};

use simplelog::{SimpleLogger, LevelFilter, Config};

fn initiate_logging() {
    let _ = SimpleLogger::init(LevelFilter::Info, Config::default());
}

fn main() {
    initiate_logging();
    let evse = EVSEHardwareImpl::new();
    start_machine(evse);
}

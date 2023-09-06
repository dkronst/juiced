
use juicelib::evse::{start_machine, EVSEHardwareImpl};
use crossbeam_channel::bounded;

use log::{info, warn, error, debug, trace};
use simplelog::{SimpleLogger, LevelFilter, Config};

fn initiate_logging() {
    let _ = SimpleLogger::init(LevelFilter::Trace, Config::default());
}

fn main() {
    initiate_logging();
    let evse = EVSEHardwareImpl::new();
    start_machine(evse);
}

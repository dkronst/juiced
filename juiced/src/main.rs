
use juicelib::evse::{start_machine, Fault, EVSEHardware, EVSEHardwareImpl};
use crossbeam_channel::bounded;

use log::{info, warn, error, debug, trace};
use simplelog::{SimpleLogger, LevelFilter, Config, TerminalMode};

fn initiate_logging() {
    let _ = SimpleLogger::init(LevelFilter::Trace, Config::default());
}

fn main() {
    initiate_logging();
    let mut evse = EVSEHardwareImpl::new();
    
    let (fault_tx, fault_rx) = bounded(1);
    let (pilot_tx, pilot_rx) = bounded(1);
    
    start_machine(pilot_rx, fault_rx, &mut evse);
}

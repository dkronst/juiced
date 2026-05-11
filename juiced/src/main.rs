use std::sync::Arc;

use juiced_status::{StatusBroker, WaveformTrigger};
use log::info;

fn main() -> ! {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();

    let broker = Arc::new(StatusBroker::new());
    let trigger = WaveformTrigger::new();
    let port: u16 = std::env::var("JUICED_WEB_PORT")
        .ok()
        .and_then(|s| s.parse().ok())
        .unwrap_or(8080);

    let reader = broker.reader();
    let trigger_for_web = trigger.clone();
    std::thread::Builder::new()
        .name("juiced-web".to_string())
        .spawn(move || juiced_web::serve(reader, trigger_for_web, port))
        .expect("spawn juiced-web thread");

    info!("dashboard at http://0.0.0.0:{}/", port);
    juicelib::evse::run_until_fatal(broker, trigger);
}

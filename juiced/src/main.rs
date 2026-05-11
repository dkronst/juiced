use juicelib::evse::run_until_fatal;

fn main() -> ! {
    env_logger::Builder::from_env(env_logger::Env::default().default_filter_or("info")).init();
    run_until_fatal();
}

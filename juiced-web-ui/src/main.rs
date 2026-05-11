//! WASM dashboard for juiced. Polls `/api/state` every second and renders
//! the current EVSE status. Depends on `juiced-status` for the wire format
//! so any divergence is a compile error.

use gloo_net::http::Request;
use gloo_timers::future::TimeoutFuture;
use juiced_status::{FsmState, Status, SupervisorPhase};
use leptos::prelude::*;
use wasm_bindgen_futures::spawn_local;

fn main() {
    console_error_panic_hook::set_once();
    leptos::mount::mount_to_body(App);
}

#[component]
fn App() -> impl IntoView {
    let (status, set_status) = signal::<Option<Status>>(None);

    // Poll /api/state every 1 s. Errors are silently dropped so a brief
    // server hiccup doesn't blank the page; the last good value stays.
    spawn_local(async move {
        loop {
            if let Ok(resp) = Request::get("/api/state").send().await {
                if let Ok(s) = resp.json::<Status>().await {
                    set_status.set(Some(s));
                }
            }
            TimeoutFuture::new(1000).await;
        }
    });

    view! {
        <main>
            <header>
                <h1>"juiced"</h1>
                <span class="uptime">
                    {move || match status.get() {
                        Some(s) => format_uptime(s.uptime_secs),
                        None => "—".to_string(),
                    }}
                </span>
            </header>
            <StatusView status=status />
            <footer>"polling /api/state every 1 s"</footer>
        </main>
    }
}

#[component]
fn StatusView(status: ReadSignal<Option<Status>>) -> impl IntoView {
    view! {
        {move || match status.get() {
            None => view! { <div class="card"><p class="muted">"Connecting…"</p></div> }.into_any(),
            Some(s) => view! {
                <div class="card">
                    <div class="state-row">
                        <span class="label">"FSM state"</span>
                        <span class=move || format!("pill {}", fsm_pill_class(s.fsm_state))>
                            {fsm_label(s.fsm_state)}
                        </span>
                    </div>
                    <p class="muted" style="margin-top:0.75rem">
                        {supervisor_label(&s.supervisor)}
                    </p>
                </div>

                <div class="card">
                    <div class="grid">
                        <div class="metric">
                            <div class="label">"Pilot voltage"</div>
                            <div class="value">
                                {format_volt_range(s.pilot_v_min, s.pilot_v_max)}
                                <span class="unit">"V"</span>
                            </div>
                        </div>
                        <div class="metric">
                            <div class="label">"Current draw"</div>
                            <div class="value">
                                {format_float(s.current_amps)}
                                <span class="unit">"A"</span>
                            </div>
                        </div>
                        <div class="metric">
                            <div class="label">"Mains (peak)"</div>
                            <div class="value">
                                {format_float(s.mains_voltage_peak)}
                                <span class="unit">"V"</span>
                            </div>
                        </div>
                        <div class="metric">
                            <div class="label">"Contactor"</div>
                            <div class="value">
                                {if s.contactor_on { "On" } else { "Off" }}
                            </div>
                        </div>
                    </div>
                </div>
            }.into_any(),
        }}
    }
}

fn fsm_label(s: FsmState) -> &'static str {
    match s {
        FsmState::Unknown => "Unknown",
        FsmState::SelfTest => "Self test",
        FsmState::Standby => "Standby",
        FsmState::VehicleDetected => "Vehicle detected",
        FsmState::StartCharging => "Starting charge",
        FsmState::Charging => "Charging",
        FsmState::StopCharging => "Stopping charge",
        FsmState::NoPower => "No power",
        FsmState::VentilationNeeded => "Ventilation needed",
        FsmState::ResetableError => "Resettable error",
        FsmState::FailedStation => "Failed",
    }
}

fn fsm_pill_class(s: FsmState) -> &'static str {
    match s {
        FsmState::Charging => "good",
        FsmState::StartCharging | FsmState::StopCharging | FsmState::SelfTest => "warn",
        FsmState::VehicleDetected | FsmState::Standby | FsmState::NoPower => "idle",
        FsmState::FailedStation | FsmState::ResetableError => "bad",
        FsmState::VentilationNeeded => "warn",
        FsmState::Unknown => "",
    }
}

fn supervisor_label(p: &SupervisorPhase) -> String {
    match p {
        SupervisorPhase::Initializing => "Supervisor: initializing".to_string(),
        SupervisorPhase::Running => "Supervisor: running".to_string(),
        SupervisorPhase::RestartingIn { seconds_remaining, reason } => {
            format!("Restarting in {} s — {}", seconds_remaining, reason)
        }
        SupervisorPhase::TerminalFault { fault } => format!("Terminal fault: {}", fault),
    }
}

fn format_float(v: Option<f32>) -> String {
    match v {
        Some(x) => format!("{:.2}", x),
        None => "—".to_string(),
    }
}

fn format_volt_range(min: Option<f32>, max: Option<f32>) -> String {
    match (min, max) {
        (Some(lo), Some(hi)) => format!("{:.1} … {:.1}", lo, hi),
        _ => "—".to_string(),
    }
}

fn format_uptime(secs: u64) -> String {
    let hours = secs / 3600;
    let minutes = (secs % 3600) / 60;
    let seconds = secs % 60;
    format!("uptime {:02}:{:02}:{:02}", hours, minutes, seconds)
}

#[cfg(test)]
mod tests {
    use super::*;
    use wasm_bindgen_test::*;

    wasm_bindgen_test_configure!(run_in_browser);

    /// The schema must round-trip identically through `juiced-status`'s serde
    /// derives on both sides of the wire. If the server ever changes the
    /// shape, this test catches it at WASM-test time.
    #[wasm_bindgen_test]
    fn test_status_struct_deserializes_from_canonical_payload() {
        let json = r#"{
            "fsm_state":"Charging",
            "supervisor":{"phase":"running"},
            "pilot_v_min":-12.0,
            "pilot_v_max":6.0,
            "current_amps":15.5,
            "mains_voltage_peak":240.0,
            "contactor_on":true,
            "uptime_secs":42,
            "last_update_unix_ms":1700000000000
        }"#;
        let parsed: Status = serde_json::from_str(json).expect("parse status");
        assert_eq!(parsed.fsm_state, FsmState::Charging);
        assert_eq!(parsed.pilot_v_max, Some(6.0));
        assert_eq!(parsed.contactor_on, true);
        assert!(matches!(parsed.supervisor, SupervisorPhase::Running));
    }
}

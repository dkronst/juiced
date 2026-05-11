//! WASM dashboard for juiced. Polls `/api/state` every second and renders
//! the current EVSE status. Depends on `juiced-status` for the wire format
//! so any divergence is a compile error.

use gloo_net::http::Request;
use gloo_timers::future::TimeoutFuture;
use juiced_status::{FsmState, Status, SupervisorPhase, Waveform};
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
            <WaveformCard status=status />
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
                            <div class="label">"Pilot peak (+)"</div>
                            <div class="value">
                                {format_signed(s.pilot_v_max)}
                                <span class="unit">"V"</span>
                            </div>
                        </div>
                        <div class="metric">
                            <div class="label">"Pilot peak (−)"</div>
                            <div class="value">
                                {format_signed(s.pilot_v_min)}
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

#[component]
fn WaveformCard(status: ReadSignal<Option<Status>>) -> impl IntoView {
    let (waveform, set_waveform) = signal::<Option<Waveform>>(None);
    let (capture_in_progress, set_capture_in_progress) = signal(false);

    // When `Status.waveform_captured_at_unix_ms` changes, GET /api/wave once.
    // Effect re-runs each time the signal it reads changes; returns the
    // previously-seen timestamp so we can compare on the next tick.
    Effect::new(move |prev: Option<Option<u64>>| {
        let ts = status.get().and_then(|s| s.waveform_captured_at_unix_ms);
        let prev_ts = prev.flatten();
        if ts.is_some() && ts != prev_ts {
            spawn_local(async move {
                if let Ok(resp) = Request::get("/api/wave").send().await {
                    if let Ok(wf) = resp.json::<Waveform>().await {
                        set_waveform.set(Some(wf));
                        set_capture_in_progress.set(false);
                    }
                }
            });
        }
        ts
    });

    let on_capture_click = move |_| {
        if capture_in_progress.get() {
            return;
        }
        set_capture_in_progress.set(true);
        spawn_local(async move {
            if Request::post("/api/wave/trigger").send().await.is_err() {
                set_capture_in_progress.set(false);
            }
        });
    };

    view! {
        <div class="card">
            <div class="state-row" style="justify-content:space-between;display:flex;width:100%">
                <span class="label">"Current-sense waveform"</span>
                <button
                    on:click=on_capture_click
                    disabled=move || capture_in_progress.get()
                    style="padding:0.4rem 0.9rem;border-radius:0.5rem;border:1px solid var(--border);background:var(--accent);color:white;font-weight:500;cursor:pointer"
                >
                    {move || if capture_in_progress.get() {
                        "Capturing… (10 s)"
                    } else {
                        "Capture (10 s)"
                    }}
                </button>
            </div>
            <div style="margin-top:1rem">
                {move || match waveform.get() {
                    None => view! {
                        <p class="muted">"Press Capture to record 50 mains cycles (10 s)."</p>
                    }.into_any(),
                    Some(wf) => render_waveform(&wf).into_any(),
                }}
            </div>
        </div>
    }
}

/// Render a captured waveform as a single SVG containing one `<polyline>`
/// per contiguous segment (the 50 capture windows have ~180 ms gaps between
/// them, which we split on so the line doesn't draw horizontal jumps).
fn render_waveform(wf: &Waveform) -> impl IntoView {
    if wf.samples.is_empty() {
        return view! { <p class="muted">"Empty waveform."</p> }.into_any();
    }

    // X domain: 0 .. last_t. Y domain: amps min..max, padded a bit.
    let t_max = wf.samples.iter().map(|s| s.t_s).fold(f32::MIN, f32::max).max(0.001);
    let mut a_min = wf.samples.iter().map(|s| s.amps).fold(f32::INFINITY, f32::min);
    let mut a_max = wf.samples.iter().map(|s| s.amps).fold(f32::NEG_INFINITY, f32::max);
    if (a_max - a_min).abs() < 0.001 {
        // Constant signal; pad so the line is visible.
        a_min -= 0.5;
        a_max += 0.5;
    }
    let a_pad = (a_max - a_min) * 0.08;
    let a_min_padded = a_min - a_pad;
    let a_max_padded = a_max + a_pad;

    // Layout
    let view_w: f32 = 1000.0;
    let view_h: f32 = 280.0;
    let margin_l: f32 = 50.0;
    let margin_r: f32 = 10.0;
    let margin_t: f32 = 10.0;
    let margin_b: f32 = 25.0;
    let plot_w = view_w - margin_l - margin_r;
    let plot_h = view_h - margin_t - margin_b;

    let map_x = |t: f32| margin_l + (t / t_max) * plot_w;
    let map_y = |a: f32| {
        margin_t + plot_h * (1.0 - (a - a_min_padded) / (a_max_padded - a_min_padded))
    };

    // Build segments: split when the time gap between consecutive samples
    // exceeds 1 ms (the within-window step is ~0.1 ms; between-window is
    // ~180 ms). Each segment is a "points" string for one <polyline>.
    let mut segments: Vec<String> = Vec::new();
    let mut current = String::new();
    let mut last_t: Option<f32> = None;
    for s in &wf.samples {
        let gap = match last_t {
            Some(prev) => s.t_s - prev,
            None => 0.0,
        };
        if gap > 0.001 && !current.is_empty() {
            segments.push(std::mem::take(&mut current));
        }
        if !current.is_empty() {
            current.push(' ');
        }
        current.push_str(&format!("{:.1},{:.1}", map_x(s.t_s), map_y(s.amps)));
        last_t = Some(s.t_s);
    }
    if !current.is_empty() {
        segments.push(current);
    }

    let zero_y = map_y(0.0);
    let zero_in_view = a_min_padded <= 0.0 && a_max_padded >= 0.0;
    let stats = format!(
        "{} samples across {} windows × {} ms; SPI {} Hz; range {:.2} A … {:.2} A",
        wf.samples.len(), wf.window_count, wf.window_ms, wf.spi_hz, a_min, a_max,
    );

    view! {
        <svg
            viewBox=format!("0 0 {} {}", view_w, view_h)
            style="width:100%;height:auto;background:rgba(0,0,0,0.03);border-radius:0.5rem"
        >
            // Y-axis labels: top, zero (if present), bottom.
            <text x="6" y=format!("{:.1}", margin_t + 10.0) style="font-size:10px;fill:var(--muted)">
                {format!("{:.1} A", a_max_padded)}
            </text>
            <text x="6" y=format!("{:.1}", margin_t + plot_h - 2.0) style="font-size:10px;fill:var(--muted)">
                {format!("{:.1} A", a_min_padded)}
            </text>
            {move || if zero_in_view {
                view! {
                    <line
                        x1=format!("{:.1}", margin_l)
                        x2=format!("{:.1}", margin_l + plot_w)
                        y1=format!("{:.1}", zero_y)
                        y2=format!("{:.1}", zero_y)
                        stroke="var(--muted)"
                        stroke-dasharray="3 3"
                        stroke-width="0.5"
                    />
                    <text
                        x=format!("{:.1}", margin_l - 4.0)
                        y=format!("{:.1}", zero_y + 3.0)
                        text-anchor="end"
                        style="font-size:10px;fill:var(--muted)"
                    >
                        "0"
                    </text>
                }.into_any()
            } else {
                ().into_any()
            }}
            // X axis baseline at the bottom of the plot.
            <line
                x1=format!("{:.1}", margin_l)
                x2=format!("{:.1}", margin_l + plot_w)
                y1=format!("{:.1}", margin_t + plot_h)
                y2=format!("{:.1}", margin_t + plot_h)
                stroke="var(--border)"
                stroke-width="0.5"
            />
            // X tick at 0 and t_max
            <text
                x=format!("{:.1}", margin_l)
                y=format!("{:.1}", margin_t + plot_h + 14.0)
                style="font-size:10px;fill:var(--muted)"
            >
                "0 s"
            </text>
            <text
                x=format!("{:.1}", margin_l + plot_w)
                y=format!("{:.1}", margin_t + plot_h + 14.0)
                text-anchor="end"
                style="font-size:10px;fill:var(--muted)"
            >
                {format!("{:.1} s", t_max)}
            </text>
            // Segments as separate polylines.
            {segments.into_iter().map(|pts| {
                view! {
                    <polyline
                        points=pts
                        fill="none"
                        stroke="var(--accent)"
                        stroke-width="1"
                    />
                }
            }).collect_view()}
        </svg>
        <p class="muted" style="margin-top:0.5rem;font-size:0.8rem">
            {stats}
        </p>
    }.into_any()
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

/// Show the sign explicitly for pilot peaks (a +9 V vs -12 V reading is
/// what tells the operator whether the pilot is oscillating).
fn format_signed(v: Option<f32>) -> String {
    match v {
        Some(x) if x >= 0.0 => format!("+{:.1}", x),
        Some(x) => format!("{:.1}", x), // negative sign already included
        None => "—".to_string(),
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
            "last_update_unix_ms":1700000000000,
            "waveform_captured_at_unix_ms":null
        }"#;
        let parsed: Status = serde_json::from_str(json).expect("parse status");
        assert_eq!(parsed.fsm_state, FsmState::Charging);
        assert_eq!(parsed.pilot_v_max, Some(6.0));
        assert_eq!(parsed.contactor_on, true);
        assert!(matches!(parsed.supervisor, SupervisorPhase::Running));
        assert_eq!(parsed.waveform_captured_at_unix_ms, None);
    }

    #[wasm_bindgen_test]
    fn test_waveform_deserializes_from_canonical_payload() {
        let json = r#"{
            "captured_at_unix_ms":1700000000000,
            "samples":[
                {"t_s":0.0,"amps":0.0},
                {"t_s":0.01,"amps":1.5},
                {"t_s":0.02,"amps":0.0}
            ],
            "window_count":1,
            "window_ms":20,
            "spi_hz":300000
        }"#;
        let wf: Waveform = serde_json::from_str(json).expect("parse waveform");
        assert_eq!(wf.captured_at_unix_ms, 1700000000000);
        assert_eq!(wf.samples.len(), 3);
        assert_eq!(wf.samples[1].amps, 1.5);
    }
}

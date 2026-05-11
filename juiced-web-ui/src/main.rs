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

    // The ADC thread captures continuously (~4 s per capture). Whenever
    // `Status.waveform_captured_at_unix_ms` advances, fetch the new
    // payload and re-render. Effect re-runs each time the status signal
    // changes; the closure receives the previously-returned value.
    Effect::new(move |prev: Option<Option<u64>>| {
        let ts = status.get().and_then(|s| s.waveform_captured_at_unix_ms);
        let prev_ts = prev.flatten();
        if ts.is_some() && ts != prev_ts {
            spawn_local(async move {
                if let Ok(resp) = Request::get("/api/wave").send().await {
                    if let Ok(wf) = resp.json::<Waveform>().await {
                        set_waveform.set(Some(wf));
                    }
                }
            });
        }
        ts
    });

    view! {
        <div class="card">
            <div class="state-row" style="justify-content:space-between;display:flex;width:100%">
                <span class="label">"Current-sense waveform"</span>
                <span class="muted" style="font-size:0.8rem">
                    {move || match waveform.get() {
                        Some(_) => "auto-captured continuously",
                        None => "waiting for first capture…",
                    }}
                </span>
            </div>
            <div style="margin-top:1rem">
                {move || match waveform.get() {
                    None => view! {
                        <p class="muted">"Capture in progress (≈ 4 s)…"</p>
                    }.into_any(),
                    Some(wf) => render_waveform(&wf).into_any(),
                }}
            </div>
        </div>
    }
}

/// Render a captured waveform as a single SVG `<polyline>`. The X-axis is
/// phase within one mains cycle (0..cycle_period_ms), computed by mod-ing
/// each sample's `t_s`. Synchronous-undersampling already places samples
/// uniformly across the cycle, so this gives a clean reconstruction of
/// the AC waveform's shape.
fn render_waveform(wf: &Waveform) -> impl IntoView {
    if wf.samples.is_empty() {
        return view! { <p class="muted">"Empty waveform."</p> }.into_any();
    }

    let cycle_s = (wf.cycle_period_ms as f32) / 1000.0;

    // Compute phase for each sample and sort by phase so the polyline draws
    // a clean reconstructed cycle (sample order in time is already monotone
    // in phase since cumulative drift is +0.1 ms/sample, but we sort
    // defensively in case of jitter or a non-monotonic capture).
    let mut points: Vec<(f32, f32)> = wf
        .samples
        .iter()
        .map(|s| {
            let phase_s = s.t_s - cycle_s * (s.t_s / cycle_s).floor();
            (phase_s, s.amps)
        })
        .collect();
    points.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal));

    // X domain: 0..cycle_s (in ms for display). Y domain: data min/max, padded.
    let mut a_min = wf.samples.iter().map(|s| s.amps).fold(f32::INFINITY, f32::min);
    let mut a_max = wf.samples.iter().map(|s| s.amps).fold(f32::NEG_INFINITY, f32::max);
    if (a_max - a_min).abs() < 0.001 {
        a_min -= 0.5;
        a_max += 0.5;
    }
    let a_pad = (a_max - a_min) * 0.08;
    let a_min_padded = a_min - a_pad;
    let a_max_padded = a_max + a_pad;

    // Layout
    let view_w: f32 = 1000.0;
    let view_h: f32 = 280.0;
    let margin_l: f32 = 56.0;
    let margin_r: f32 = 12.0;
    let margin_t: f32 = 10.0;
    let margin_b: f32 = 28.0;
    let plot_w = view_w - margin_l - margin_r;
    let plot_h = view_h - margin_t - margin_b;

    let map_x = move |phase_s: f32| margin_l + (phase_s / cycle_s) * plot_w;
    let map_y = move |a: f32| {
        margin_t + plot_h * (1.0 - (a - a_min_padded) / (a_max_padded - a_min_padded))
    };

    let poly_points: String = points
        .iter()
        .map(|(p, a)| format!("{:.1},{:.1}", map_x(*p), map_y(*a)))
        .collect::<Vec<_>>()
        .join(" ");

    let zero_y = map_y(0.0);
    let zero_in_view = a_min_padded <= 0.0 && a_max_padded >= 0.0;
    let cycle_ms_display = wf.cycle_period_ms as f32;
    let stats = format!(
        "{} samples × {:.1} ms cycle (50 Hz mains); SPI {} Hz; range {:.2} A … {:.2} A",
        wf.samples_per_cycle, cycle_ms_display, wf.spi_hz, a_min, a_max,
    );

    view! {
        <svg
            viewBox=format!("0 0 {} {}", view_w, view_h)
            style="width:100%;height:auto;background:rgba(0,0,0,0.03);border-radius:0.5rem"
        >
            // Y-axis tick labels: top, bottom (and zero, if it's in range).
            <text x="6" y=format!("{:.1}", margin_t + 10.0) style="font-size:10px;fill:var(--muted)">
                {format!("{:.2} A", a_max_padded)}
            </text>
            <text x="6" y=format!("{:.1}", margin_t + plot_h - 2.0) style="font-size:10px;fill:var(--muted)">
                {format!("{:.2} A", a_min_padded)}
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
            // X-axis labels: 0 ms, mid, full cycle.
            <text
                x=format!("{:.1}", margin_l)
                y=format!("{:.1}", margin_t + plot_h + 14.0)
                style="font-size:10px;fill:var(--muted)"
            >
                "0 ms"
            </text>
            <text
                x=format!("{:.1}", margin_l + plot_w / 2.0)
                y=format!("{:.1}", margin_t + plot_h + 14.0)
                text-anchor="middle"
                style="font-size:10px;fill:var(--muted)"
            >
                {format!("{:.0} ms (½ cycle)", cycle_ms_display / 2.0)}
            </text>
            <text
                x=format!("{:.1}", margin_l + plot_w)
                y=format!("{:.1}", margin_t + plot_h + 14.0)
                text-anchor="end"
                style="font-size:10px;fill:var(--muted)"
            >
                {format!("{:.0} ms (1 cycle)", cycle_ms_display)}
            </text>
            <polyline
                points=poly_points
                fill="none"
                stroke="var(--accent)"
                stroke-width="1.5"
            />
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
                {"t_s":0.0201,"amps":1.5},
                {"t_s":0.0402,"amps":0.0}
            ],
            "samples_per_cycle":3,
            "cycle_period_ms":20,
            "spi_hz":300000
        }"#;
        let wf: Waveform = serde_json::from_str(json).expect("parse waveform");
        assert_eq!(wf.captured_at_unix_ms, 1700000000000);
        assert_eq!(wf.samples.len(), 3);
        assert_eq!(wf.samples[1].amps, 1.5);
        assert_eq!(wf.samples_per_cycle, 3);
        assert_eq!(wf.cycle_period_ms, 20);
    }
}

//! HTTP dashboard for juiced. Read-only state, plus a single control input
//! (`POST /api/wave/trigger`) for one-shot diagnostic waveform captures.
//!
//! `serve(reader, trigger, port)` blocks the calling thread. The web exporter
//! is a pure consumer of `juiced_status::StatusReader` for state, and writes
//! to `juiced_status::WaveformTrigger` *only* via the explicit trigger
//! endpoint. It has no dependency on `juicelib` and no awareness of the FSM
//! internals.
//!
//! Routes:
//!   GET  /                              -> index.html (embedded)
//!   GET  /api/state                     -> JSON snapshot of Status
//!   GET  /api/wave                      -> JSON of the latest Waveform (404 if none)
//!   POST /api/wave/trigger              -> 202 Accepted; flips the WaveformTrigger
//!   GET  /static/juiced-web-ui_bg.wasm  -> WASM binary (trunk output)
//!   GET  /static/juiced-web-ui.js       -> trunk-generated WASM loader
//!   GET  /static/app.css                -> stylesheet

use juiced_status::{StatusReader, WaveformTrigger};
use log::{error, info, warn};
use tiny_http::{Header, Method, Response, Server};

mod assets;

/// Bind to `0.0.0.0:port` and serve until the server is dropped (never, in
/// production). Pass `0` to bind an ephemeral port.
pub fn serve(reader: StatusReader, trigger: WaveformTrigger, port: u16) {
    serve_with_port_callback(reader, trigger, port, |_| {})
}

/// Like `serve`, but invokes `on_bound` once the server has a real port.
/// Used by tests that pass `port = 0` and need to discover the ephemeral
/// port before they can issue HTTP requests.
pub fn serve_with_port_callback<F>(
    reader: StatusReader,
    trigger: WaveformTrigger,
    port: u16,
    on_bound: F,
) where
    F: FnOnce(u16),
{
    let addr = format!("0.0.0.0:{}", port);
    let server = match Server::http(&addr) {
        Ok(s) => s,
        Err(e) => {
            error!("juiced-web: failed to bind to {}: {}", addr, e);
            return;
        }
    };
    let bound_port = server.server_addr().to_ip().map(|a| a.port()).unwrap_or(0);
    info!("juiced-web: listening on http://0.0.0.0:{}/", bound_port);
    on_bound(bound_port);

    for request in server.incoming_requests() {
        let path = request.url().split('?').next().unwrap_or("/").to_string();
        let method = request.method().clone();
        match (method, path.as_str()) {
            (Method::Get, "/") | (Method::Get, "/index.html") => {
                respond_bytes(request, 200, "text/html; charset=utf-8", assets::INDEX_HTML)
            }
            (Method::Get, "/api/state") => match serde_json::to_vec(&reader.snapshot()) {
                Ok(json) => respond_bytes(request, 200, "application/json", &json),
                Err(e) => {
                    error!("snapshot serialization failed: {}", e);
                    respond_bytes(request, 500, "text/plain", b"snapshot serialization failed")
                }
            },
            (Method::Get, "/api/wave") => match reader.latest_waveform() {
                Some(wf) => match serde_json::to_vec(&wf) {
                    Ok(json) => respond_bytes(request, 200, "application/json", &json),
                    Err(e) => {
                        error!("waveform serialization failed: {}", e);
                        respond_bytes(
                            request,
                            500,
                            "text/plain",
                            b"waveform serialization failed",
                        )
                    }
                },
                None => respond_bytes(request, 404, "text/plain", b"no waveform captured yet"),
            },
            (Method::Post, "/api/wave/trigger") => {
                trigger.request();
                respond_bytes(request, 202, "text/plain", b"capture requested")
            }
            (Method::Get, "/static/juiced-web-ui_bg.wasm") => {
                respond_bytes(request, 200, "application/wasm", assets::APP_WASM)
            }
            (Method::Get, "/static/juiced-web-ui.js") => {
                respond_bytes(request, 200, "application/javascript", assets::APP_JS)
            }
            (Method::Get, "/static/app.css") => {
                respond_bytes(request, 200, "text/css", assets::APP_CSS)
            }
            _ => respond_bytes(request, 404, "text/plain", b"not found"),
        }
    }
}

fn respond_bytes(request: tiny_http::Request, status: u16, content_type: &str, body: &[u8]) {
    let header = Header::from_bytes(b"Content-Type".as_ref(), content_type.as_bytes())
        .expect("static content-type strings are valid");
    let response = Response::from_data(body.to_vec())
        .with_status_code(status)
        .with_header(header);
    if let Err(e) = request.respond(response) {
        warn!("response send failed: {}", e);
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use juiced_status::{
        FsmState, Status, StatusBroker, SupervisorPhase, Waveform, WaveformSample,
    };
    use std::thread;
    use std::time::Duration;

    fn spawn(broker: &StatusBroker, trigger: &WaveformTrigger) -> u16 {
        let reader = broker.reader();
        let trig = trigger.clone();
        let (port_tx, port_rx) = std::sync::mpsc::channel();
        thread::spawn(move || {
            serve_with_port_callback(reader, trig, 0, move |bound| {
                let _ = port_tx.send(bound);
            });
        });
        port_rx
            .recv_timeout(Duration::from_secs(2))
            .expect("server failed to bind")
    }

    fn get(port: u16, path: &str) -> (u16, String, String) {
        let url = format!("http://127.0.0.1:{}{}", port, path);
        let resp = ureq::get(&url).call().unwrap_or_else(|e| match e {
            ureq::Error::Status(_, r) => r,
            other => panic!("HTTP transport error: {:?}", other),
        });
        let status = resp.status();
        let ct = resp.header("Content-Type").unwrap_or("").to_string();
        let body = resp.into_string().unwrap_or_default();
        (status, ct, body)
    }

    fn post_empty(port: u16, path: &str) -> u16 {
        let url = format!("http://127.0.0.1:{}{}", port, path);
        let resp = ureq::post(&url).send_bytes(&[]).unwrap_or_else(|e| match e {
            ureq::Error::Status(_, r) => r,
            other => panic!("HTTP transport error: {:?}", other),
        });
        resp.status()
    }

    #[test]
    fn test_api_state_returns_serialized_status() {
        let broker = StatusBroker::new();
        broker.set_fsm_state(FsmState::Charging);
        broker.update_pilot_voltage(-12.0, 6.0);
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);

        let (status, ct, body) = get(port, "/api/state");
        assert_eq!(status, 200);
        assert!(ct.starts_with("application/json"), "wrong content-type: {}", ct);
        let parsed: Status = serde_json::from_str(&body).expect("json parse");
        assert_eq!(parsed.fsm_state, FsmState::Charging);
        assert_eq!(parsed.pilot_v_max, Some(6.0));
    }

    #[test]
    fn test_root_returns_html_with_wasm_loader() {
        let broker = StatusBroker::new();
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);
        let (status, ct, body) = get(port, "/");
        assert_eq!(status, 200);
        assert!(ct.starts_with("text/html"), "wrong content-type: {}", ct);
        assert!(
            body.contains("type=\"module\"") || body.contains("type='module'"),
            "expected ES-module loader in / response"
        );
    }

    #[test]
    fn test_static_assets_served() {
        let broker = StatusBroker::new();
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);

        let (wasm_status, wasm_ct, _) = get(port, "/static/juiced-web-ui_bg.wasm");
        assert_eq!(wasm_status, 200);
        assert_eq!(wasm_ct, "application/wasm");

        let (css_status, css_ct, _) = get(port, "/static/app.css");
        assert_eq!(css_status, 200);
        assert!(css_ct.starts_with("text/css"), "wrong content-type: {}", css_ct);

        let (js_status, js_ct, _) = get(port, "/static/juiced-web-ui.js");
        assert_eq!(js_status, 200);
        assert!(js_ct.starts_with("application/javascript"), "wrong content-type: {}", js_ct);
    }

    #[test]
    fn test_unknown_path_returns_404() {
        let broker = StatusBroker::new();
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);
        let (status, _, _) = get(port, "/nope");
        assert_eq!(status, 404);
    }

    #[test]
    fn test_serve_reflects_live_status_updates() {
        let broker = StatusBroker::new();
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);

        let (_, _, body1) = get(port, "/api/state");
        let snap1: Status = serde_json::from_str(&body1).unwrap();
        assert_eq!(snap1.fsm_state, FsmState::Unknown);

        broker.set_fsm_state(FsmState::Charging);
        broker.set_supervisor(SupervisorPhase::Running);

        let (_, _, body2) = get(port, "/api/state");
        let snap2: Status = serde_json::from_str(&body2).unwrap();
        assert_eq!(snap2.fsm_state, FsmState::Charging);
        assert!(matches!(snap2.supervisor, SupervisorPhase::Running));
    }

    fn fake_waveform(ts: u64) -> Waveform {
        Waveform {
            captured_at_unix_ms: ts,
            window_count: 2,
            window_ms: 20,
            spi_hz: 300_000,
            samples: vec![
                WaveformSample { t_s: 0.000, amps: 0.0 },
                WaveformSample { t_s: 0.010, amps: 1.5 },
                WaveformSample { t_s: 0.020, amps: 0.0 },
            ],
        }
    }

    #[test]
    fn test_api_wave_returns_404_before_any_capture() {
        let broker = StatusBroker::new();
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);
        let (status, _, _) = get(port, "/api/wave");
        assert_eq!(status, 404);
    }

    #[test]
    fn test_api_wave_returns_published_waveform() {
        let broker = StatusBroker::new();
        broker.publish_waveform(fake_waveform(1700000000123));
        let trigger = WaveformTrigger::new();
        let port = spawn(&broker, &trigger);

        let (status, ct, body) = get(port, "/api/wave");
        assert_eq!(status, 200);
        assert!(ct.starts_with("application/json"), "wrong content-type: {}", ct);
        let parsed: Waveform = serde_json::from_str(&body).expect("json parse");
        assert_eq!(parsed.captured_at_unix_ms, 1700000000123);
        assert_eq!(parsed.samples.len(), 3);
        assert_eq!(parsed.samples[1].amps, 1.5);
    }

    #[test]
    fn test_api_wave_trigger_sets_request_flag_and_returns_202() {
        let broker = StatusBroker::new();
        let trigger = WaveformTrigger::new();
        assert!(!trigger.is_requested());
        let port = spawn(&broker, &trigger);

        let status = post_empty(port, "/api/wave/trigger");
        assert_eq!(status, 202);
        assert!(trigger.is_requested(), "trigger flag should be set after POST");
    }
}

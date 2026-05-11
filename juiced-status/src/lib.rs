//! Shared status surface for juiced.
//!
//! Every exporter (web dashboard, USB serial, Home Assistant, …) depends on
//! this crate and *only* this crate to read or write EVSE status. The crate
//! is intentionally free of hardware, threading-spawn, and HTTP code so it
//! compiles to `wasm32-unknown-unknown` for the browser-side dashboard.
//!
//! The data types (`Status`, `FsmState`, `SupervisorPhase`) are available on
//! every target. The runtime broker (`StatusBroker`, `StatusReader`) is only
//! compiled for native targets because it depends on `Arc<RwLock>` and
//! `Instant`; the WASM client never touches those.

use serde::{Deserialize, Serialize};

/// Mirror of `juicelib::evse::EVSEMachineState` for wire-format purposes.
/// The `From` impl converting from the rust_fsm-derived type lives in
/// `juicelib` so that this crate has no dependency on the FSM internals.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, Deserialize)]
pub enum FsmState {
    Unknown,
    SelfTest,
    Standby,
    VehicleDetected,
    StartCharging,
    Charging,
    StopCharging,
    NoPower,
    VentilationNeeded,
    ResetableError,
    FailedStation,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
#[serde(tag = "phase", rename_all = "snake_case")]
pub enum SupervisorPhase {
    Initializing,
    Running,
    RestartingIn {
        seconds_remaining: u64,
        reason: String,
    },
    TerminalFault {
        fault: String,
    },
}

/// One raw current-sense sample inside a diagnostic capture.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct WaveformSample {
    /// Seconds since the start of the *entire* capture (window 0, sample 0 at t=0).
    pub t_s: f32,
    pub amps: f32,
}

/// A diagnostic snapshot of the current-sense waveform, reconstructed via
/// **synchronous undersampling**: we sample the current channel at the
/// mains period (1/50 s) plus a small per-sample drift `dt / samples`, so
/// after `samples` samples we've walked exactly one full mains cycle of
/// phase. Each `WaveformSample.t_s` is the real elapsed time; the phase
/// within a 50 Hz cycle is `t_s mod (cycle_period_ms / 1000)`.
///
/// This trades capture time (~4 s) for waveform fidelity even at the
/// MCP3008's slow ~12 k samples/s SPI rate: every phase in the cycle gets
/// sampled exactly once with high temporal resolution.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Waveform {
    /// Milliseconds since the Unix epoch when the capture *finished*. The
    /// UI compares this against the broker's previous value to detect a
    /// fresh capture.
    pub captured_at_unix_ms: u64,
    pub samples: Vec<WaveformSample>,
    /// Number of samples spread across one mains cycle's worth of phase.
    pub samples_per_cycle: u32,
    /// Base sampling period in ms (= mains period; 20 ms for 50 Hz).
    pub cycle_period_ms: u32,
    /// SPI clock used during sampling; recorded so the operator can sanity
    /// check the effective sample rate from the data.
    pub spi_hz: u32,
}

/// Snapshot of the EVSE status at a point in time. This is the on-the-wire
/// shape served by `juiced-web` and consumed by `juiced-web-ui`.
#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct Status {
    pub fsm_state: FsmState,
    pub supervisor: SupervisorPhase,
    pub pilot_v_min: Option<f32>,
    pub pilot_v_max: Option<f32>,
    pub current_amps: Option<f32>,
    pub mains_voltage_peak: Option<f32>,
    pub contactor_on: bool,
    pub uptime_secs: u64,
    /// Milliseconds since the Unix epoch when any field last changed.
    pub last_update_unix_ms: u64,
    /// `Some(ts)` if a diagnostic waveform has been captured; ts is the
    /// `captured_at_unix_ms` of the most recent `Waveform`. The UI polls
    /// this to detect a fresh capture without GETting the full payload.
    pub waveform_captured_at_unix_ms: Option<u64>,
}

impl Status {
    /// Build a default Status. Used as the broker's initial value and as
    /// a placeholder by the WASM client before it has any data.
    pub fn initial() -> Self {
        Status {
            fsm_state: FsmState::Unknown,
            supervisor: SupervisorPhase::Initializing,
            pilot_v_min: None,
            pilot_v_max: None,
            current_amps: None,
            mains_voltage_peak: None,
            contactor_on: false,
            uptime_secs: 0,
            last_update_unix_ms: 0,
            waveform_captured_at_unix_ms: None,
        }
    }
}

// ---------------------------------------------------------------------------
// Broker: native targets only.
// ---------------------------------------------------------------------------

#[cfg(not(target_arch = "wasm32"))]
mod broker_impl {
    use super::*;
    use std::sync::{Arc, RwLock};
    use std::time::{Instant, SystemTime, UNIX_EPOCH};

    fn unix_now_ms() -> u64 {
        SystemTime::now()
            .duration_since(UNIX_EPOCH)
            .map(|d| d.as_millis() as u64)
            .unwrap_or(0)
    }

    /// Shared state container behind the broker and reader. Held by both.
    struct Inner {
        status: RwLock<Status>,
        waveform: RwLock<Option<Waveform>>,
        start: Instant,
    }

    /// Single-writer end of the status surface. Writers (FSM, ADC thread,
    /// supervisor) hold a `StatusBroker` and call narrow update methods.
    ///
    /// `StatusBroker` is `Clone` so writers in spawned threads can hold
    /// their own handles without an outer `Arc`.
    #[derive(Clone)]
    pub struct StatusBroker {
        inner: Arc<Inner>,
    }

    impl StatusBroker {
        pub fn new() -> Self {
            Self {
                inner: Arc::new(Inner {
                    status: RwLock::new(Status::initial()),
                    waveform: RwLock::new(None),
                    start: Instant::now(),
                }),
            }
        }

        /// Get a cheap-to-clone read handle.
        pub fn reader(&self) -> StatusReader {
            StatusReader {
                inner: Arc::clone(&self.inner),
            }
        }

        fn write<F: FnOnce(&mut Status)>(&self, f: F) {
            let mut s = self
                .inner
                .status
                .write()
                .unwrap_or_else(|p| p.into_inner());
            f(&mut s);
            s.last_update_unix_ms = unix_now_ms();
        }

        pub fn set_fsm_state(&self, state: FsmState) {
            self.write(|s| s.fsm_state = state);
        }

        pub fn set_supervisor(&self, phase: SupervisorPhase) {
            self.write(|s| s.supervisor = phase);
        }

        pub fn update_pilot_voltage(&self, min: f32, max: f32) {
            self.write(|s| {
                s.pilot_v_min = Some(min);
                s.pilot_v_max = Some(max);
            });
        }

        pub fn update_current_amps(&self, amps: f32) {
            self.write(|s| s.current_amps = Some(amps));
        }

        pub fn update_mains_voltage(&self, peak: f32) {
            self.write(|s| s.mains_voltage_peak = Some(peak));
        }

        pub fn set_contactor_on(&self, on: bool) {
            self.write(|s| s.contactor_on = on);
        }

        /// Store a diagnostic waveform and update `Status.waveform_captured_at_unix_ms`
        /// so UI clients can detect a fresh capture without fetching the full payload.
        pub fn publish_waveform(&self, waveform: Waveform) {
            let ts = waveform.captured_at_unix_ms;
            {
                let mut slot = self
                    .inner
                    .waveform
                    .write()
                    .unwrap_or_else(|p| p.into_inner());
                *slot = Some(waveform);
            }
            self.write(|s| s.waveform_captured_at_unix_ms = Some(ts));
        }
    }

    impl Default for StatusBroker {
        fn default() -> Self {
            Self::new()
        }
    }

    /// Read-only handle. Consumers (web, USB, HA) hold this and call
    /// `snapshot` whenever they need to publish a sample.
    #[derive(Clone)]
    pub struct StatusReader {
        inner: Arc<Inner>,
    }

    impl StatusReader {
        pub fn snapshot(&self) -> Status {
            let mut s = self
                .inner
                .status
                .read()
                .unwrap_or_else(|p| p.into_inner())
                .clone();
            // `uptime_secs` is computed from the broker's birthday so each
            // snapshot reflects current wall time without writers needing
            // to update it on every tick.
            s.uptime_secs = self.inner.start.elapsed().as_secs();
            s
        }

        /// Latest diagnostic waveform, if any has been captured.
        pub fn latest_waveform(&self) -> Option<Waveform> {
            self.inner
                .waveform
                .read()
                .unwrap_or_else(|p| p.into_inner())
                .clone()
        }
    }
}

#[cfg(not(target_arch = "wasm32"))]
pub use broker_impl::{StatusBroker, StatusReader};

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;
    use std::thread;
    use std::time::Duration;

    /// Compile-time assertion helper for `Clone + Send + Sync`.
    fn assert_clone_send_sync<T: Clone + Send + Sync + 'static>() {}

    #[test]
    fn test_default_status_has_safe_initial_values() {
        let broker = StatusBroker::new();
        let snap = broker.reader().snapshot();
        assert_eq!(snap.fsm_state, FsmState::Unknown);
        assert!(matches!(snap.supervisor, SupervisorPhase::Initializing));
        assert_eq!(snap.pilot_v_min, None);
        assert_eq!(snap.pilot_v_max, None);
        assert_eq!(snap.current_amps, None);
        assert_eq!(snap.mains_voltage_peak, None);
        assert_eq!(snap.contactor_on, false);
    }

    #[test]
    fn test_set_fsm_state_round_trips() {
        let broker = StatusBroker::new();
        broker.set_fsm_state(FsmState::Charging);
        assert_eq!(broker.reader().snapshot().fsm_state, FsmState::Charging);
    }

    #[test]
    fn test_update_pilot_voltage_sets_min_max_and_advances_clock() {
        let broker = StatusBroker::new();
        let before = broker.reader().snapshot().last_update_unix_ms;
        // Sleep > 1 ms so the millisecond clock provably advances.
        thread::sleep(Duration::from_millis(5));
        broker.update_pilot_voltage(-12.0, 6.0);
        let snap = broker.reader().snapshot();
        assert_eq!(snap.pilot_v_min, Some(-12.0));
        assert_eq!(snap.pilot_v_max, Some(6.0));
        assert!(
            snap.last_update_unix_ms > before,
            "last_update should advance after an update; before={} after={}",
            before, snap.last_update_unix_ms
        );
    }

    #[test]
    fn test_uptime_grows_monotonically() {
        let broker = StatusBroker::new();
        let t0 = broker.reader().snapshot().uptime_secs;
        thread::sleep(Duration::from_millis(1100));
        let t1 = broker.reader().snapshot().uptime_secs;
        assert!(t1 >= t0 + 1, "uptime should grow: {} -> {}", t0, t1);
    }

    #[test]
    fn test_status_serde_round_trip() {
        let broker = StatusBroker::new();
        broker.set_fsm_state(FsmState::Charging);
        broker.update_pilot_voltage(-12.0, 6.0);
        broker.update_current_amps(15.5);
        broker.update_mains_voltage(240.0);
        broker.set_contactor_on(true);
        broker.set_supervisor(SupervisorPhase::Running);

        let snap = broker.reader().snapshot();
        let json = serde_json::to_string(&snap).expect("serialize");
        let parsed: Status = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(parsed.fsm_state, snap.fsm_state);
        assert_eq!(parsed.pilot_v_min, snap.pilot_v_min);
        assert_eq!(parsed.pilot_v_max, snap.pilot_v_max);
        assert_eq!(parsed.current_amps, snap.current_amps);
        assert_eq!(parsed.mains_voltage_peak, snap.mains_voltage_peak);
        assert_eq!(parsed.contactor_on, snap.contactor_on);
        assert!(matches!(parsed.supervisor, SupervisorPhase::Running));
    }

    #[test]
    fn test_concurrent_writes_dont_deadlock() {
        let broker = Arc::new(StatusBroker::new());
        let mut handles = vec![];
        for w in 0..4 {
            let b = Arc::clone(&broker);
            handles.push(thread::spawn(move || {
                for i in 0..1000 {
                    b.update_pilot_voltage(-12.0, (w * 1000 + i) as f32 * 0.001);
                }
            }));
        }
        for _ in 0..4 {
            let reader = broker.reader();
            handles.push(thread::spawn(move || {
                for _ in 0..1000 {
                    let _ = reader.snapshot();
                }
            }));
        }
        let start = std::time::Instant::now();
        for h in handles {
            h.join().expect("worker panicked");
        }
        assert!(
            start.elapsed() < Duration::from_secs(2),
            "concurrent writes/reads too slow; likely deadlock contention"
        );
    }

    #[test]
    fn test_status_reader_is_clone_send_sync() {
        assert_clone_send_sync::<StatusReader>();
    }

    // ----- Diagnostic waveform capture -----

    fn fake_waveform(ts: u64) -> Waveform {
        Waveform {
            captured_at_unix_ms: ts,
            samples_per_cycle: 6,
            cycle_period_ms: 20,
            spi_hz: 300_000,
            samples: vec![
                WaveformSample { t_s: 0.0000, amps: 0.0 },
                WaveformSample { t_s: 0.0201, amps: 1.5 },
                WaveformSample { t_s: 0.0402, amps: 0.0 },
                WaveformSample { t_s: 0.0603, amps: -1.5 },
                WaveformSample { t_s: 0.0804, amps: 0.0 },
                WaveformSample { t_s: 0.1005, amps: 1.5 },
            ],
        }
    }

    #[test]
    fn test_initial_status_has_no_waveform_timestamp() {
        let broker = StatusBroker::new();
        let snap = broker.reader().snapshot();
        assert_eq!(snap.waveform_captured_at_unix_ms, None);
        assert!(broker.reader().latest_waveform().is_none());
    }

    #[test]
    fn test_waveform_round_trips_through_broker() {
        let broker = StatusBroker::new();
        let wf = fake_waveform(1700000000123);
        broker.publish_waveform(wf.clone());

        let got = broker.reader().latest_waveform().expect("waveform present");
        assert_eq!(got.captured_at_unix_ms, wf.captured_at_unix_ms);
        assert_eq!(got.samples_per_cycle, 6);
        assert_eq!(got.cycle_period_ms, 20);
        assert_eq!(got.spi_hz, 300_000);
        assert_eq!(got.samples.len(), 6);
        assert_eq!(got.samples[1].amps, 1.5);
    }

    #[test]
    fn test_waveform_publish_advances_status_timestamp() {
        let broker = StatusBroker::new();
        broker.publish_waveform(fake_waveform(1700000000123));
        let snap = broker.reader().snapshot();
        assert_eq!(snap.waveform_captured_at_unix_ms, Some(1700000000123));
    }

    #[test]
    fn test_waveform_serde_round_trip() {
        let wf = fake_waveform(1700000000456);
        let json = serde_json::to_string(&wf).expect("serialize");
        let parsed: Waveform = serde_json::from_str(&json).expect("deserialize");
        assert_eq!(parsed.captured_at_unix_ms, wf.captured_at_unix_ms);
        assert_eq!(parsed.samples.len(), wf.samples.len());
        assert_eq!(parsed.samples[1].amps, wf.samples[1].amps);
    }
}

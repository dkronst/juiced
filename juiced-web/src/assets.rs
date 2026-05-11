//! Static assets served by `juiced-web`.
//!
//! Compiled into the binary at build time from `../juiced-web-ui/dist/`,
//! which is produced by `trunk build --release`. See `build.rs` for the
//! dependency tracking that re-triggers compilation when UI sources change.

const UI_DIST: &str = "../juiced-web-ui/dist";

pub const INDEX_HTML: &[u8] = include_bytes!(concat!("../../juiced-web-ui/dist/index.html"));
pub const APP_WASM: &[u8] = include_bytes!(concat!("../../juiced-web-ui/dist/juiced-web-ui_bg.wasm"));
pub const APP_JS: &[u8] = include_bytes!(concat!("../../juiced-web-ui/dist/juiced-web-ui.js"));
pub const APP_CSS: &[u8] = include_bytes!(concat!("../../juiced-web-ui/dist/app.css"));

#[allow(dead_code)]
const _: &str = UI_DIST; // keep the docstring's path string referenced

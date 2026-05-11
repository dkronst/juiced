//! Triggers a `cargo` rebuild whenever the WASM UI sources change. Doesn't
//! invoke `trunk` itself — leaves that as an explicit step (run
//! `cd juiced-web-ui && trunk build --release`) so callers control when
//! they pay the 30 s WASM build cost.
//!
//! Prints a clear error if `dist/` is missing so the `include_bytes!` in
//! `src/assets.rs` doesn't blow up with a cryptic message.

use std::path::Path;

fn main() {
    let manifest = Path::new(env!("CARGO_MANIFEST_DIR"));
    let ui_root = manifest.join("../juiced-web-ui");
    let dist = ui_root.join("dist");

    // Re-run this build script (and therefore re-evaluate include_bytes!) when
    // any of these change.
    for sub in &[
        "src",
        "assets",
        "Cargo.toml",
        "Trunk.toml",
        "index.html",
        "dist",
    ] {
        println!("cargo:rerun-if-changed={}", ui_root.join(sub).display());
    }

    for needed in &["index.html", "juiced-web-ui_bg.wasm", "juiced-web-ui.js", "app.css"] {
        if !dist.join(needed).exists() {
            panic!(
                "\n  juiced-web: missing {}/{}.\n  \
                 Build the WASM UI first:\n    \
                 (cd juiced-web-ui && trunk build --release)\n",
                dist.display(),
                needed
            );
        }
    }
}

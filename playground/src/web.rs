//! The scene in the page's URL, after the `#`.

use eframe::wasm_bindgen::JsValue;

/// The URL after the `#`.
pub fn hash() -> String {
    let hash = web_sys::window().and_then(|window| window.location().hash().ok());
    hash.map(|hash| hash.trim_start_matches('#').to_owned())
        .unwrap_or_default()
}

/// Sets the URL after the `#`, without adding to the history: the back button doesn't step
/// through every drag.
pub fn set_hash(hash: &str) {
    if let Some(history) = web_sys::window().and_then(|window| window.history().ok()) {
        let _ = history.replace_state_with_url(&JsValue::NULL, "", Some(&format!("#{hash}")));
    }
}

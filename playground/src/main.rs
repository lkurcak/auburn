//! The auburn playground: try auburn2d's queries by hand, on the desktop and in the browser.
//!
//! `cargo run -p auburn-playground` opens it, and `trunk serve` in `playground/` serves it to
//! a browser. On the desktop, a playground link, or the part after its `#`, opens that scene:
//! `cargo run -p auburn-playground -- 'q=collides&m=box'`.

mod app;
mod scene;
#[cfg(target_arch = "wasm32")]
mod web;

use eframe::egui;

#[cfg(not(target_arch = "wasm32"))]
fn main() -> eframe::Result {
    let scene = std::env::args().nth(1).map(|link| {
        scene::Scene::from_link(&link).unwrap_or_else(|| {
            eprintln!("usage: auburn-playground [SCENE]");
            eprintln!();
            eprintln!("SCENE is a playground link, or the part after its #.");
            std::process::exit(2)
        })
    });
    let options = eframe::NativeOptions {
        viewport: egui::ViewportBuilder::default()
            .with_title("auburn playground")
            .with_inner_size([1200.0, 760.0]),
        ..Default::default()
    };
    eframe::run_native(
        "auburn playground",
        options,
        Box::new(|cc| Ok(Box::new(start(cc, scene)))),
    )
}

#[cfg(target_arch = "wasm32")]
fn main() {
    use eframe::wasm_bindgen::JsCast as _;

    wasm_bindgen_futures::spawn_local(async {
        let canvas = web_sys::window()
            .and_then(|window| window.document())
            .and_then(|document| document.get_element_by_id("playground"))
            .and_then(|canvas| canvas.dyn_into::<web_sys::HtmlCanvasElement>().ok())
            .expect("index.html has a <canvas id=\"playground\">");
        let started = eframe::WebRunner::new()
            .start(
                canvas,
                eframe::WebOptions::default(),
                Box::new(|cc| Ok(Box::new(start(cc, None)))),
            )
            .await;
        if let Err(error) = started {
            web_sys::console::error_1(&error);
        }
    });
}

fn start(cc: &eframe::CreationContext<'_>, scene: Option<scene::Scene>) -> app::App {
    // The canvas's colours are picked for a dark background.
    cc.egui_ctx.set_theme(egui::Theme::Dark);
    app::App::new(scene)
}

# Playground

One app for trying auburn2d's queries by hand. It's a debugger for us and a demo for
anyone, on the desktop (`cargo run -p auburn-playground`) and in the browser.

## Decided (agent, 27 Nov 2026)

- **From scratch, not a port of [auburn2d-vis](https://github.com/lkurcak/auburn2d-vis).**
  That's 12 near-identical Bevy binaries (six queries, each for `Vec2` and for
  `Transform2d`) on auburn2d 0.2.3 and Bevy 0.19. Keep its ideas: a shape on the mouse,
  number keys to switch it, a different colour for a hit.
- **egui (eframe), not Bevy.** One codebase runs natively and on wasm. The side panel
  (query, shapes, the numbers) comes free, the download and the build are a fraction of
  Bevy's with rendering, and the strokes are antialiased. Bevy users already have
  `auburn2d/examples/bevy-integration.rs`.
- **A workspace member at `playground/`, with `publish = false`.** forge's CI runs clippy,
  the tests and the docs with `--workspace --all-features` on Linux, macOS and Windows, so
  the playground keeps compiling through every API change. `cargo publish --workspace`
  skips it.

## Scope

- A scene with a point, a ball, a box and a polygon, each with a `Transform2d`
  (translation, rotation, scale). Drag a shape to move it; rotate and scale it with a
  handle or the scroll wheel. One of them is the moving shape.
- The query, picked in the side panel:
  - `collides`: shapes that hit are filled.
  - `penetrates`: the push-out vector, and the moved shape as a ghost.
  - `toi`, `toiae`, `tttoi`, `tttoiae`: drag a velocity arrow; ghosts at entry and exit.
  - The side panel shows the numbers each query returns.
- **Copy as test**: copies the selected pair as the body of a Rust test, so a bug found by
  hand becomes a test. On the web the scene is also kept in the URL's hash, so a link
  brings it back.
- It shows what the library returns, bugs included: it's a debugger first.

## Done when

- `cargo run -p auburn-playground` opens it, and it builds for `wasm32-unknown-unknown`
  and runs in a browser (`trunk serve` in `playground/`).
- The forge CI passes with it in the workspace.

# auburn playground

Try auburn2d's queries by hand: a point, a ball, a box and a polygon, one of them moving.
Pick a query and drag the shapes around. Shapes that hit are filled, and the side panel shows
each query's exact answer, bugs included: it's a debugger first.

- **Online:** <https://lkurcak.github.io/auburn/>, built from main.
- **On the desktop:** `cargo run -p auburn-playground`. A playground link, or the part after
  its `#`, opens that scene: `cargo run -p auburn-playground -- 'q=collides&m=box'`.
- **In the browser, from source:** `trunk serve` in this directory, then open
  <http://127.0.0.1:8080>. Install [trunk](https://trunkrs.dev) and the `wasm32-unknown-unknown`
  target first. The page's URL keeps the scene, so a link brings it back.

**Copy as test** copies the moving shape and another one as a Rust test that asserts what
auburn2d answers now. A bug found by hand goes into `auburn2d/tests/regressions.rs`, with the
right answer in place of the wrong one.

It isn't published to crates.io. It's a member of the workspace, so CI builds, lints and tests
it with every change to auburn2d. `.github/workflows/pages.yml` builds it for the browser on
pull requests that touch it or auburn2d, and publishes it on GitHub Pages when they're merged.

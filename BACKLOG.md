# Backlog

## Now

1. Publish the playground on GitHub Pages and link it from the README. Its `pages.yml` has to
   be listed in auburn's `own_workflows` in forge-config, and Pages set to deploy from Actions.

## Later

- Cast result with time, hit normal and contact point. `CastResult` is commented out in `relation/cast.rs`; sliding along walls needs the normal.
- Capsule shape in 2D and 3D.
- Segment shape and a ray query API.
- More shapes in the enums: `Poly2d` in `Shape2d`; a 3D convex hull/polyhedron shape and a `Shape3d` enum.
- Cleanup: remove `derive_type::Wrapper` (deprecated since 0.1.0), the commented-out code, and duplicated iteration constants.
- Benchmarks (criterion) against parry2d/3d, to back the "fast" claim.
- [Performance](backlog/performance.md), after the benchmarks.
- Fix the root README's CI badge: it points at `rust.yml`, but the workflow is `ci.yml`, so the image 404s.
- [Docs: READMEs, missing docs, positioning](backlog/docs.md)
- `move_and_slide` helper for kinematic characters. Needs the cast result.
- Small bevy plugin: collider components and a query `SystemParam`. The bevy example currently checks every pair.
- Simple broadphase: a uniform grid or sort-and-sweep.
- Playground: step through GJK and EPA, drawing the simplex and the polytope at each
  iteration. Needs hooks in the algorithms.
- Playground in 3D.
- Playground: polygons other than regular ones, with vertices to drag. Only convex ones:
  `Poly2d::new` panics on others in debug builds.
- Save a playground scene as a PNG (egui_kittest snapshots), so an agent can look at a
  failing randomized case.
- glam 0.34, only once bevy depends on it. `forge deps` reports "Cargo.toml would have to change".
- no_std. Tried and reverted in 2024; the CI no-std job exists but is skipped. Low priority.
- `cargo test --no-default-features --features bevy` doesn't compile: `tests/randomized` in both
  crates imports `glam` directly. CI doesn't run that combination.
- `From<Transform2d> for bevy Isometry2d` drops the scale without saying so: document it, or
  convert only when the scale is 1.
- With `--all-features`, cargo warns that the optional `glam` dependency is unused in both crates,
  because bevy's `Vec3` wins (cargo's `unused_dependencies` lint).
- Bring 3D's ray cast fixes to 2D's `shape_cast`: balls as a centre and a radius
  (`rounded_shape_cast`), and a hit test that allows for the angle the ray meets the set at.
  Ball–Poly2d still goes through it and passes at 200×. Before Ball–Box2d got an exact cast,
  it got 1 turned Ball–Box2d case in 6 000 wrong at huge sizes, at 20×: an exit 1% late.
- [3D `shape_cast` takes near-misses for grazes](backlog/ray-cast-near-miss.md).

## Done

- Playground (`cargo run -p auburn-playground`, or `trunk serve` in `playground/`): drag a point, a ball, a box and a polygon, see each query's answers and ghosts, copy a pair as a test, and keep the scene in the page's URL (#PR)
- Released 0.4.0 on crates.io: auburn2d and auburn3d, with every fix and the any-two-colliders casts since 0.3.3 (#33)
- Any two colliders cast, whatever their shapes and transformations, mixed ones too: one generic impl per cast, with exact formulas for points, balls, boxes and polygons (in 3D: points, balls, and points against boxes) and the GJK ray cast for the rest; `Shape::primitive` and `rotation_and_scale` tell when a shape keeps its form; the ~70 hand-written per-pair impls are gone (#32)
- 3D casts without an exact formula, every pair with a `Box3d`, read the overlap's span off one GJK ray cast, `algorithm::shape_cast`, with a ball as its centre and radius: they no longer stop at t = 1 or stall, exit times are exact and time-traveling ones go below 0; `Transform3d` Point and Ball pairs use the exact `Vec3` casts; conservative advancement is gone (#31)
- 2D casts without an exact formula, Ball–Poly2d so far, read the overlap's span off one GJK ray cast, `algorithm::shape_cast`: they no longer stop at t = 1 or stall, exit times are exact and time-traveling ones go below 0; conservative advancement is gone from 2D (#30)
- 2D bevy `Transform` uses its own x and y scale on the plane z = 0, so uneven scales and mirrors work and z no longer leaks in; `inverse` is the matrix inverse in 2D and 3D, and converting to `Transform2d`/`Transform3d` keeps an even scale exactly (#29)
- `From<ScaleTranslate2d> for bevy Isometry2d` is gone: its body was `todo!()`, so it could only panic (#28)
- `Poly2d` casts are one swept SAT that ignores winding and has no absolute tolerances, so tiny and clockwise polygons work; `regular` is counter-clockwise, `area` works, an empty polygon is a point at its origin, and debug builds check convexity (#27)
- Ball and point casts read off one overlap span: no NaN at rest, negative time-traveling times in 3D, tiny velocities, Point–Point through-paths; the at-rest answers are documented and every pair gives them (#26)
- `Transform2d` ball casts scale each radius once, by its own |scale|, and ball `toiae` gives the exit when starting inside; box casts take |scale| too (#25)

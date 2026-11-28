# Backlog

## Now

1. The same cast result in 3D: `a.cast(&b, vel)` and `CastHit`, as 2D has them (#37), with the
   randomized check against the true contact.
1. `move_and_slide` helper for kinematic characters. Needs the cast result.

## Later

- Capsule shape in 2D and 3D.
- Segment shape and a ray query API.
- More shapes in the enums: `Poly2d` in `Shape2d`; a 3D convex hull/polyhedron shape and a `Shape3d` enum.
- Cleanup: remove `derive_type::Wrapper` (deprecated since 0.1.0), the commented-out code, and duplicated iteration constants.
- Benchmarks (criterion) against parry2d/3d, to back the "fast" claim.
- [Performance](backlog/performance.md), after the benchmarks.
- [Docs: READMEs, missing docs, positioning](backlog/docs.md)
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
- 2D cast hits through support functions alone, with no ball to round by, as for a ball
  under an uneven bevy scale or a custom curved shape: the normal comes from the ray cast's
  last separating line, so at a curved contact it can be off by about the square root of the
  cast's tolerance. The randomized suite samples no such shape.
- [3D `shape_cast` takes near-misses for grazes](backlog/ray-cast-near-miss.md).

## Done

- 2D `a.cast(&b, vel)` gives a `CastHit`: `toi`'s time, the contact's normal and a point where the shapes touch, or the shortest way out if they overlap already; exact for the pairs with formulas. Balls go through 3D's ray cast as a centre and a radius, which settles on the nearest point. The playground draws the hit (#37)
- The root README's CI badge points at `ci.yml`, the workflow that runs, instead of `rust.yml`, whose badge 404'd (#36)
- Playground online at <https://lkurcak.github.io/auburn/>, linked from the READMEs: `pages.yml` builds it for the browser on pull requests and publishes it from main (#35)
- Playground (`cargo run -p auburn-playground`, or `trunk serve` in `playground/`): drag a point, a ball, a box and a polygon, see each query's answers and ghosts, copy a pair as a test, and keep the scene in the page's URL (#34)
- Released 0.4.0 on crates.io: auburn2d and auburn3d, with every fix and the any-two-colliders casts since 0.3.3 (#33)
- Any two colliders cast, whatever their shapes and transformations, mixed ones too: one generic impl per cast, with exact formulas for points, balls, boxes and polygons (in 3D: points, balls, and points against boxes) and the GJK ray cast for the rest; `Shape::primitive` and `rotation_and_scale` tell when a shape keeps its form; the ~70 hand-written per-pair impls are gone (#32)
- 3D casts without an exact formula, every pair with a `Box3d`, read the overlap's span off one GJK ray cast, `algorithm::shape_cast`, with a ball as its centre and radius: they no longer stop at t = 1 or stall, exit times are exact and time-traveling ones go below 0; `Transform3d` Point and Ball pairs use the exact `Vec3` casts; conservative advancement is gone (#31)
- 2D casts without an exact formula, Ball–Poly2d so far, read the overlap's span off one GJK ray cast, `algorithm::shape_cast`: they no longer stop at t = 1 or stall, exit times are exact and time-traveling ones go below 0; conservative advancement is gone from 2D (#30)
- 2D bevy `Transform` uses its own x and y scale on the plane z = 0, so uneven scales and mirrors work and z no longer leaks in; `inverse` is the matrix inverse in 2D and 3D, and converting to `Transform2d`/`Transform3d` keeps an even scale exactly (#29)
- `From<ScaleTranslate2d> for bevy Isometry2d` is gone: its body was `todo!()`, so it could only panic (#28)

# Backlog

## Now

1. Remove `From<ScaleTranslate2d> for bevy Isometry2d` (bevy feature): its body is `todo!()`, so it can only panic. Removing it is technically breaking.
1. [Bevy `Transform` with uneven scale](backlog/bevy-uneven-scale.md)
1. [One generic shape cast replacing conservative advancement](backlog/generic-shape-cast.md)
1. [Cast coverage through the generic cast](backlog/cast-coverage.md)
1. Release 0.4.0.
1. [Playground: try the 2D queries by hand, on the desktop and in the browser](backlog/playground.md)
1. Publish the playground on GitHub Pages and link it from the README. Its `pages.yml` has to
   be listed in auburn's `own_workflows` in forge-config, and Pages set to deploy from Actions.

## Later

- Cast result with time, hit normal and contact point. `CastResult` is commented out in `relation/cast.rs`; sliding along walls needs the normal. Needs the generic shape cast.
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
  iteration. Needs hooks in the algorithms, so after the generic shape cast.
- Playground in 3D.
- Save a playground scene as a PNG (egui_kittest snapshots), so an agent can look at a
  failing randomized case.
- glam 0.34, only once bevy depends on it. `forge deps` reports "Cargo.toml would have to change".
- no_std. Tried and reverted in 2024; the CI no-std job exists but is skipped. Low priority.
- `cargo test --no-default-features --features bevy` doesn't compile: `tests/randomized` in both
  crates imports `glam` directly. CI doesn't run that combination.
- With `--all-features`, cargo warns that the optional `glam` dependency is unused in both crates,
  because bevy's `Vec3` wins (cargo's `unused_dependencies` lint).

## Done

- `Poly2d` casts are one swept SAT that ignores winding and has no absolute tolerances, so tiny and clockwise polygons work; `regular` is counter-clockwise, `area` works, an empty polygon is a point at its origin, and debug builds check convexity (#27)
- Ball and point casts read off one overlap span: no NaN at rest, negative time-traveling times in 3D, tiny velocities, Point–Point through-paths; the at-rest answers are documented and every pair gives them (#26)
- `Transform2d` ball casts scale each radius once, by its own |scale|, and ball `toiae` gives the exit when starting inside; box casts take |scale| too (#25)
- 2D EPA returns the shortest support distance it finds, so deep, near-concentric balls get their depth and direction (#24)
- 3D GJK searches from the part of its simplex nearest the origin, so it no longer cycles and misses shallow overlaps (#23)
- 3D EPA orients faces by their winding and keeps its polytope closed, so balls get their depth (#21)
- `Collider::support` maps directions through the transpose, exact far from the origin; GJK finds points on flat boxes (#20)
- GJK and EPA tolerances scale with the shapes (#18)
- Reversed 3D `toiae`/`tttoiae` keep entry before exit (#17)
- Randomized suites against an exact ground truth, failing checks ignored by backlog item (#16)

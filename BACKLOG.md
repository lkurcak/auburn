# Backlog

## Now

1. [3D EPA returns zero or wrong vectors](backlog/epa-3d.md)
1. [2D EPA points the wrong way on deep, near-concentric overlaps](backlog/epa-2d.md)
1. [`Transform2d` casts mishandle scale](backlog/transform2d-scale.md)
1. [Cast edge cases: NaN, negative times, zero and tiny velocities](backlog/cast-edge-cases.md)
1. [`Poly2d` robustness: winding, epsilons, `area`, empty polygons](backlog/poly2d-robustness.md)
1. Remove `From<ScaleTranslate2d> for bevy Isometry2d` (bevy feature): its body is `todo!()`, so it can only panic. Removing it is technically breaking.
1. [Bevy `Transform` with uneven scale](backlog/bevy-uneven-scale.md)
1. [One generic shape cast replacing conservative advancement](backlog/generic-shape-cast.md)
1. [Cast coverage through the generic cast](backlog/cast-coverage.md)
1. Release 0.4.0.

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
- glam 0.34, only once bevy depends on it. `forge deps` reports "Cargo.toml would have to change".
- no_std. Tried and reverted in 2024; the CI no-std job exists but is skipped. Low priority.
- `cargo test --no-default-features --features bevy` doesn't compile: `tests/randomized` in both
  crates imports `glam` directly. CI doesn't run that combination.

## Done

- `Collider::support` maps directions through the transpose, exact far from the origin; GJK finds points on flat boxes (#20)
- GJK and EPA tolerances scale with the shapes (#18)
- Reversed 3D `toiae`/`tttoiae` keep entry before exit (#17)
- Randomized suites against an exact ground truth, failing checks ignored by backlog item (#16)

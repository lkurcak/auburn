# Backlog

## Now

1. [3D reversed `toiae`/`tttoiae` swap entry and exit](backlog/toiae-swap-3d.md)
2. [GJK and EPA tolerances must scale with the shapes](backlog/gjk-tolerances.md)
3. [`Collider::support` loses precision far from the origin](backlog/support-direction.md)
4. [3D EPA returns zero or wrong vectors](backlog/epa-3d.md)
5. [2D EPA points the wrong way on deep, near-concentric overlaps](backlog/epa-2d.md)
6. [`Transform2d` casts mishandle scale](backlog/transform2d-scale.md)
7. [Cast edge cases: NaN, negative times, zero and tiny velocities](backlog/cast-edge-cases.md)
8. [`Poly2d` robustness: winding, epsilons, `area`, empty polygons](backlog/poly2d-robustness.md)
9. Remove `From<ScaleTranslate2d> for bevy Isometry2d` (`auburn2d/src/transformation/scale_translate2d.rs`, bevy feature): it is two `todo!()`s. Deleting it is technically breaking, but it can only panic.
10. [Bevy `Transform` with uneven scale](backlog/bevy-uneven-scale.md)
11. [One generic shape cast replacing conservative advancement](backlog/generic-shape-cast.md)
12. [Cast coverage through the generic cast](backlog/cast-coverage.md)
13. Release 0.4.0. Ask the user first: releases are tag-triggered through forge and publish to crates.io.

## Later

- Cast result with time, hit normal and contact point. `CastResult` is commented out in `relation/cast.rs`; sliding along walls needs the normal. Needs the generic shape cast.
- Capsule shape in 2D and 3D.
- Segment shape and a ray query API.
- More shapes in the enums: `Poly2d` in `Shape2d`; a 3D convex hull/polyhedron shape and a `Shape3d` enum.
- Cleanup: remove `derive_type::Wrapper` (deprecated since 0.1.0), the commented-out code, and duplicated iteration constants.
- Benchmarks (criterion) against parry2d/3d, to back the "fast" claim.
- [Performance](backlog/performance.md), after the benchmarks.
- [Docs: READMEs, CI badge, missing docs, positioning](backlog/docs.md)
- `move_and_slide` helper for kinematic characters. Needs the cast result.
- Small bevy plugin: collider components and a query `SystemParam`. The bevy example currently checks every pair.
- Simple broadphase: a uniform grid or sort-and-sweep.
- glam 0.34, only once bevy depends on it. `forge deps` reports "Cargo.toml would have to change".
- no_std. Tried and reverted in 2024; the CI no-std job exists but is skipped. Low priority.

## Done

- Land the randomized suite as permanent tests (#16): `tests/randomized` and `tests/regressions.rs` in both crates, with failing checks `#[ignore]`d by backlog item.

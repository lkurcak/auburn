# 3D `shape_cast` takes near-misses for grazes

`auburn3d::algorithm::shape_cast` stalls short of a box that a point's path misses by 0.0029
(the box is about 3 across) and reports a graze, entry = exit. It accepts the stall because
what it reached is within `STALL_TOLERANCE`, 1e-3 of the set's size. The repro is
`shape_cast_near_miss` in `auburn3d/tests/regressions.rs`.

- At `AUBURN_RANDOMIZED_SCALE=200`, before Point–Box3d had an exact cast, 6 of about 11
  million Point–Box3d cases went wrong, in both orders and under several transformations
  (Transform3d at huge sizes, bevy Transform, mixed ones). Box–Box3d and the rounded
  Ball–Box3d cast had none.
- Points against boxes have an exact cast now, so the randomized suite no longer reaches
  this. Box–Box3d and shapes with only a support function still go through it.
- To find more cases, run the randomized Point–Box3d pairs through `shape_cast` directly at
  200× (they were in `tests/randomized/main.rs` before the exact cast).
- Where to look: why the ray cast stalls this far out. Rounding in the support points is
  around 1e-7 of the size, so a stall 1e-3 away comes from the iteration, not from rounding.
  Start with the flat-tetrahedron test and the angle-aware hit test from #31.
- 2D's `shape_cast` passes at 200× on everything that still goes through it, but has the
  same stall tolerance.

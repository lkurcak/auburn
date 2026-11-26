# GJK tolerances must scale with the shapes

Reproduced on cf06cd2; line numbers are approximate.

- 2D: `Ball(0.01)@(0.00123,-0.0007)` vs `Ball(0.01)@(0.016786,-0.016256)` (gap 0.002) gives
  a→b true, b→a false.
  - Cause: an absolute `1e-10` threshold on the triple product's squared length at
    `auburn2d/src/algorithm/gjk.rs:64`. The triple product scales with size³.
  - Changing the threshold to `== 0.0` removed every false positive in fuzzing.
- 3D: `Ball(1e-6)@0` vs `Ball(1e-6)@(1e-5,0,0)` returns true.
  - Cause: the early exit `first_point.length_squared() < 1e-10` at
    `auburn3d/src/algorithm/gjk.rs:196`.
- Grep both `gjk.rs` files for other absolute epsilons.
- Make 2D and 3D agree on degenerate cases. Coincident points and zero-size shapes collide in
  3D but not in 2D.

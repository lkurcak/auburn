# GJK and EPA tolerances must scale with the shapes

Reproduced on cf06cd2; line numbers are approximate.

- 2D: `Ball(0.01)@(0.00123,-0.0007)` vs `Ball(0.01)@(0.016786,-0.016256)` (gap 0.002) gives
  a→b true, b→a false.
  - Cause: an absolute `1e-10` threshold on the triple product's squared length at
    `auburn2d/src/algorithm/gjk.rs:64`. The triple product scales with size³.
  - Changing the threshold to `== 0.0` removed every false positive in fuzzing.
- 3D: `Ball(1e-6)@0` vs `Ball(1e-6)@(1e-5,0,0)` returns true.
  - Cause: the early exit `first_point.length_squared() < 1e-10` at
    `auburn3d/src/algorithm/gjk.rs:196`.
- Grep both `gjk.rs` files for other absolute epsilons. EPA has them too:
  `EPA_TOLERANCE = 1e-4` in both `epa.rs` files is about the size of the shapes at 1e-4
  scale, so penetration vectors are wrong there, and at 1e4 scale f32 can't resolve it, so
  3D box pairs fail 40% of the time. `algorithm/toi.rs` has `TOLERANCE = 1e-4` and
  `vel.length_squared() < 1e-10`.
- Casts that check for a starting overlap with `collides` inherit the bug: 2D Vec2
  Ball–Box2d `toi` returns `Some(0)` for tiny shapes that don't touch.
- Make 2D and 3D agree on degenerate cases. Coincident points and zero-size shapes collide in
  3D but not in 2D.

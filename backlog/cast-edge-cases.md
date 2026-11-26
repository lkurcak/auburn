# Cast edge cases

Reproduced on cf06cd2.

- (a) 2D `tttoi`/`tttoiae` return NaN at zero velocity: `Point@0 .tttoi(Ball(1)@(5,0), 0)`
  gives `Some(NaN)`; it should be None.
  - Decide and document what an overlap at zero velocity returns.
  - Files: `vec2/` point and ball casts and `transform2d/ball_point.rs`, which have no
    `a == 0` guard. `Transform2d` Ball `toiae` at rest returns `(0, NaN)`.
- (b) 3D `tttoi`/`tttoiae` never return negative times, because they just call
  `toi`/`toiae`. `Ball(1)@0 .tttoi(Point@(0.5,0,0), X)` gives 0; should be −0.5.
- (c) 3D zero velocity while overlapping: `Ball(1)@0 .toi(Point@(0.5,0,0), 0)` gives None;
  should be `Some(0)`.
- (d) 3D tiny velocity: vel (1e-6,0,0) vs a ball at (3,0,0) gives 833333; should be ~1e6.
  Cause: absolute `a < 1e-10` in `vec3/ball_point.rs` `solve_quadratic_*`.
- (e) 2D Point–Point `toi` is always None. Decide on the behavior and document it.

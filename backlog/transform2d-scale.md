# `Transform2d` casts mishandle scale

In `auburn2d/src/interactions/optimized/transform2d/`. Reproduced on cf06cd2; line numbers
are approximate.

- (a) Ball–Ball applies scale twice. `Ball(1)@identity` vs `Ball(1)` at pos (5,0) with
  scale 2: toi = `Some(0)`, should be 2.0. `collides` correctly says false. See
  `ball_ball.rs` ~45-49.
- (b) `Ball.toi(Point)` uses the point's scale for the radius. Ball r=1, scale 2 at 0, point
  at (3,0): `Some(2.0)`, should be 1.0. See `ball_point.rs` ~13 and ~43-49.
- (c) `toiae` exit time is 0 when starting inside. A point inside `Ball(1)` gives (0,0);
  should be (0,1). See `ball_point.rs` ~57-58.
- (d) Negative scale. With scale −1, Ball–Ball, Box–Box and Ball–Box toi give 5.0; should be
  3.0. The signed scale is used as a radius or extent (`ball_ball.rs:46`, `ball_box.rs:16`,
  `box_box.rs:42-43`). Use |scale|.

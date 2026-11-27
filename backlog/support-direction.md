# `Collider::support` loses precision far from the origin

Found by the randomized suite (the `far` configuration, near (1e4, -1e4)).

- Cause: `Collider::support` (`auburn2d/src/collider.rs`) maps the direction with
  `inverse().apply(dir) - inverse().apply_to_origin()`, which adds and then subtracts the
  translation. Far from the origin that cancels catastrophically, and short directions come
  out wrong.
- Repro: `Ball(1)@(1e4,-1e4) .support((1e-4,0))` gives the centre (10000,-10000), not
  (10001,-10000). At the origin it gives (1,0).
- Since #18 no query fails because of it: GJK's search directions are now as long as the
  shapes, so they survive the cancellation at 1e4. The `far` tests and
  `far_from_origin_ball_contains_point` pass and are no longer ignored. Add the repro above as
  a test of `support` itself.
- Fix: map the direction through the linear part only. [Bevy uneven scale](bevy-uneven-scale.md)
  wants the same change (the transpose of the linear part), so the two could land together.
- 3D has the same code (`auburn3d/src/collider.rs`).

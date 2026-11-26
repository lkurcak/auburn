# `Collider::support` loses precision far from the origin

Found by the randomized suite (the `far` configuration, near (1e4, -1e4)).

- `Ball(0.9429619)@(9998.233,-9997.66)` vs `Point@(9997.541,-9997.661)` overlap by 0.25, but
  `ball.collides(point)` is false while `point.collides(ball)` is true, and
  `ball.penetrates(point)` is None.
- Cause: `Collider::support` (`auburn2d/src/collider.rs`) maps the direction with
  `inverse().apply(dir) - inverse().apply_to_origin()`, which adds and then subtracts the
  translation. Far from the origin that cancels catastrophically, so GJK's short search
  directions come out wrong.
- Fix: map the direction through the linear part only. [Bevy uneven scale](bevy-uneven-scale.md)
  wants the same change (the transpose of the linear part).
- 3D has the same code (`auburn3d/src/collider.rs`); its `far` tests don't fail yet.

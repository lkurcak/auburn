# Performance

Do this after the benchmarks exist.

- `Collider::support` computes `isometry.inverse()` twice per support call.
- A bevy `Transform` is converted to `Transform2d` on every apply/inverse.
- Compute the inverse once per query.

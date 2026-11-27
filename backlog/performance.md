# Performance

Do this after the benchmarks exist.

- `Collider::support` computes `isometry.inverse()` twice per support call.
- A bevy `Transform` is converted to `Transform2d` on every apply/inverse.
- Compute the inverse once per query.
- 3D EPA looks for the closest face by scanning every face, removed ones too; a heap would
  do. Balls take 20 to 60 iterations to converge, and 2.6% of the randomized cases stop at
  the cap of 64.

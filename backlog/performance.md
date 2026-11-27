# Performance

Do this after the benchmarks exist.

- `Collider::support` computes `isometry.inverse()` twice per support call.
- A bevy `Transform` is converted to `Transform2d` on every apply/inverse.
- Compute the inverse once per query.
- The `Transform2d` Poly2d casts collect each polygon's world-space vertices into a `Vec` on
  every call. The `Vec2` ones work in the polygon's frame and don't.
- 3D EPA looks for the closest face by scanning every face, removed ones too; a heap would
  do. Balls take 20 to 60 iterations to converge, and 2.6% of the randomized cases stop at
  the cap of 64.
- 2D EPA scans every edge too. Nearly concentric balls (centres less than about 1% of the
  radii apart) use most or all of its 64 iterations: a near-circle is refined evenly, so
  the closest edge never gets within the tolerance, though the answer is right to about
  1e-6. Treating each shape as a polygon plus a radius would make balls exact in a step.

# Performance

Do this after the benchmarks exist.

- The default `Transformation2d::support_direction` computes `inverse()` on every support
  call, for user types that don't implement it. Compute the inverse once per query.
- Poly2d casts under a rotation or a scale collect the polygon's turned vertices into a `Vec`
  on every call. Under a translation alone they don't.
- Most casts got faster in #32, but three are slower than the hand-written casts they
  replaced: Ball–Box2d `toi` (about 2×, as it now finds the exit too), Box2d–Poly2d (1.2×)
  and two Box2d at different angles (1.35×).
- 3D EPA looks for the closest face by scanning every face, removed ones too; a heap would
  do. Balls take 20 to 60 iterations to converge, and 2.6% of the randomized cases stop at
  the cap of 64.
- 2D EPA scans every edge too. Nearly concentric balls (centres less than about 1% of the
  radii apart) use most or all of its 64 iterations: a near-circle is refined evenly, so
  the closest edge never gets within the tolerance, though the answer is right to about
  1e-6. Treating each shape as a polygon plus a radius would make balls exact in a step.

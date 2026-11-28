# Performance

The benchmarks (`cargo bench -p auburn-benches`) cover points, balls and boxes only: time
a change here with a benchmark of the pair it speeds up.

- The default `Transformation2d::support_direction` computes `inverse()` on every support
  call, for user types that don't implement it. Compute the inverse once per query.
- Poly2d casts under a rotation or a scale collect the polygon's turned vertices into a `Vec`
  on every call. Under a translation alone they don't.
- Most casts got faster in #32, but two are slower than the hand-written casts they
  replaced: Box2d–Poly2d (1.2×) and two Box2d at different angles (1.35×). Ball–Box2d `toi` is
  a basic pair: see [its item](fast-casts.md).
- 3D GJK under `Transform3d` takes about 7 times as long as at `Vec3`: `collides` for a point
  and a ball took 146 ns against 21 before it had a formula (benchmarks under load, 28 Nov
  2026). Each support call turns the direction and the point by the quaternion. Two boxes
  turned different ways still go through it.
- 3D EPA looks for the closest face by scanning every face, removed ones too; a heap would
  do. Balls take 20 to 60 iterations to converge, and 2.6% of the randomized cases stopped at
  the cap of 64. Built-in balls no longer go through it, only those under an uneven scale or
  against a shape of the game's own.
- 2D EPA scans every edge too. Nearly concentric balls (centres less than about 1% of the
  radii apart) use most or all of its 64 iterations: a near-circle is refined evenly, so
  the closest edge never gets within the tolerance, though the answer is right to about
  1e-6. As in 3D, only balls under an uneven scale or against a shape of the game's own get
  there: treating such a shape as its core plus a radius, as the casts do, would make them
  exact in a step.

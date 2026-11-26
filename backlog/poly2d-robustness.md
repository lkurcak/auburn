# `Poly2d` robustness

Reproduced on cf06cd2.

- `Poly2d::regular` builds clockwise polygons, but `point_in_convex_polygon`
  (`interactions/optimized/poly2d_utils.rs:13`) assumes counter-clockwise and uses an absolute
  `1e-6`:
  - a point at a hexagon's centre with zero-velocity `toi` gives None;
  - at scale 1e-3, a point 0.0006 outside is reported inside.
- Fix: make `regular()` counter-clockwise, normalize winding in `new()` or make the test
  winding-agnostic, and use a relative epsilon. The rest of `poly2d_utils.rs` has absolute
  epsilons too (`1e-6` and `1e-10` in the segment test, `1e-10` on velocities); at 1e-4
  scale Point–Poly2d, Poly2d–Box2d and Poly2d–Poly2d casts go wrong.
- Also:
  - `Poly2d::area` is `unimplemented!()`; implement it with the shoelace formula;
  - an empty `Poly2d::default()` panics in `support` (`points[0]`);
  - validate convexity and winding with `debug_assert`.

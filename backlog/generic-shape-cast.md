# One generic shape cast in 3D, replacing conservative advancement

In `auburn3d/src/algorithm/toi.rs`. Do what 2D does since #30: `auburn2d::algorithm::shape_cast`
is a GJK ray cast (van den Bergen) against the Minkowski difference `b - a` that gives the
overlap's span over all of time, and pairs read their four casts off it with
`casts_from_span!`. 3D needs a tetrahedron simplex in place of the triangle. Reproduced on
cf06cd2.

- Today's fallback stops at t∈[0,1] and stalls once the shapes' projections along the motion
  overlap; `estimate_distance` measures only that gap. Repro:
  `Box3d(1)@0 .toi(Box3d(1)@(10,0,0), X)` gives None.
- Affects every 3D cast involving Box3d, and all 3D `Transform3d` casts.
- Also: exit times are quantized to 0.01, and the unbounded variant never goes below 0.
- Settle the time range: toi ∈ [0,∞), as the docs already claim, and tttoi ∈ (−∞,∞).
- Keep the conventions documented in `relation/cast.rs`. The at-rest answers hold today through
  an early return in each conservative-advancement function, and the randomized at-rest checks
  require them for every pair.
- `Transform3d` scale is uniform, so its Point and Ball pairs could use the exact `Vec3` casts
  on world-space balls, as 2D does since #25, instead of the generic cast.
- From 2D: start each ray cast on the plane where the line enters the difference's slab along
  the motion, so its first step decides whether the line misses. Tolerances are relative to
  the simplex and to its coordinates' rounding. Near a rounded corner, the support function
  can keep returning the far end of a flat edge, so the ray cast stops once a step neither
  moves the point nor brings the simplex closer, and decides with a looser tolerance. Without
  that, about 1 cast in 10⁶ ran out of iterations.

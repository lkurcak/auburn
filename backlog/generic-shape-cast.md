# One generic shape cast replacing conservative advancement

In `algorithm/toi.rs`, 2D and 3D. Use GJK ray cast against the Minkowski difference
(van den Bergen). Reproduced on cf06cd2.

- Today's fallback stops at t∈[0,1] and stalls once the shapes' projections along the motion
  overlap; `estimate_distance` measures only that gap. Repros:
  - `Ball(1)@0 .toi(square Poly2d@(10,0), X)` gives None (Ball–Box2d gives 8.0);
  - square Poly2d at (3,1.5) with vel (2,0) gives `Some(1.0)`, true 0.567;
  - 3D `Box3d(1)@0 .toi(Box3d(1)@(10,0,0), X)` gives None.
- Affects 2D Ball↔Poly2d, every 3D cast involving Box3d, and all 3D `Transform3d` casts.
- Also: exit times are quantized to 0.01, and the unbounded variant never goes below 0.
- Settle the time range everywhere: toi ∈ [0,∞), as the docs already claim, and
  tttoi ∈ (−∞,∞).
- Probably split into a 2D PR and a 3D PR.
- Keep the conventions documented in `relation/cast.rs`. The at-rest answers already hold,
  through an early return in each conservative-advancement function, and the randomized
  at-rest checks require them for every pair.
- `Transform3d` scale is uniform, so its Point and Ball pairs could use the exact `Vec3` casts
  on world-space balls, as 2D does since #25, instead of the generic cast.

# 2D EPA points the wrong way on deep, near-concentric overlaps

Reproduced on the commit of #18.

- `Ball(0.7204)@(0.1655,0.1422)` vs `Ball(4.7942)@0` gives (-4.886,-2.042), about 160° off;
  0.426 of overlap remains after applying it.
- It gives up after `EPA_MAX_ITERATIONS` (30) in `auburn2d/src/algorithm/epa.rs` without
  meeting its tolerance.
- Curved shapes also leave 0.5–2% residual overlap, though EPA has stopped on a relative
  tolerance since #18. Concentric `Ball(1)`/`Ball(2)` gives |p| = 2.985.
- The same pairs fail at every scale (`penetrates::transform2d_tiny_epa` and `_huge_epa`).
- Rarely, polygons too: `Poly2d` scaled by −2.5 against a `Point` gave |p| = 0.45 for a depth
  of 0.23.

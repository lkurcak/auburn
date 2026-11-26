# 2D EPA points the wrong way on deep, near-concentric overlaps

Reproduced on cf06cd2; line numbers are approximate.

- `Ball(0.7204)@(0.1655,0.1422)` vs `Ball(4.7942)@0` gives (-4.886,-2.042), about 160° off;
  0.426 of overlap remains after applying it.
- It gives up after 30 iterations (`auburn2d/src/algorithm/epa.rs` ~243-266).
- Curved shapes also leave 0.5–2% residual overlap. Consider stopping on a tolerance instead.
  Concentric `Ball(1)`/`Ball(2)` gives |p| = 2.985.
- Rarely, polygons too: `Poly2d` scaled by −2.5 against a `Point` gave |p| = 0.45 for a depth
  of 0.23.

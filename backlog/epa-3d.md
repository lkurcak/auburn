# 3D EPA returns zero or wrong vectors

Affects 20–30% of ball/point overlaps and about 1% of box pairs. Reproduced on cf06cd2; line
numbers are approximate.

- `Ball(1.8047013)@(2.0084305,1.0236154,-2.4085774)` vs `Point@(2.7466345,0.9463959,-2.1651053)`
  gives `Some((0,0,0))`; true depth 1.0236.
- Concentric `Ball(1)`/`Ball(2)` gives |p| = 2.62; it should be 3.
- File: `auburn3d/src/algorithm/epa.rs`. Suspects:
  - zero normals from degenerate faces (~124-133);
  - the `Vec3::ZERO` fallback (~116-121);
  - the GJK early exit handing EPA a 1-point simplex instead of a tetrahedron (related to
    [GJK tolerances](gjk-tolerances.md)).

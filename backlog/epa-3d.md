# 3D EPA returns zero or wrong vectors

Affects about half of ball/point overlaps and 2–3% of box pairs (the randomized suite at 20
times the cases). Reproduced on the commit of #18.

- `Ball(1.8047013)@(2.0084305,1.0236154,-2.4085774)` vs `Point@(2.7466345,0.9463959,-2.1651053)`
  gives about `(6.5e-13,-6.1e-13,0)`; true depth 1.0236.
- Concentric `Ball(1)`/`Ball(2)` gives |p| = 2.62; it should be 3.
- File: `auburn3d/src/algorithm/epa.rs`. Suspects:
  - degenerate faces: `compute_face_normal` leaves their normal unnormalized, so the face
    sits at distance ~0 and wins `find_closest_face`. The near-zero answers look like this;
  - the `Vec3::ZERO` fallback at the end of `epa`.
- No longer a suspect: GJK's early exit now only fires when the first support point is
  exactly the origin, i.e. for shapes that touch.
- Failure rates move by a few percent with the tetrahedron GJK hands over, but not with the
  stopping tolerance.
- `box_box_penetrates` (`tests/definitions.rs`) and `box_box_penetration_depth` pass with
  `--ignored` since `Collider::support` maps directions exactly, only because GJK hands over
  a different tetrahedron. They stay ignored until EPA is fixed.

# 3D GJK cycles and misses shallow overlaps

GJK returns `None` for some Ball/Box3d overlaps, so `collides` says false and `penetrates`
returns `None`. Found while fixing 3D EPA (#21). It affects 1 to 3 in 20000 Ball/Box3d overlaps,
up to 0.049 deep, in the randomized `penetrates` suite at 20 times the cases (`vec3`, `pos3d`
and `transform3d_mirrored`). The `collides` suite draws far fewer overlaps, so it hasn't hit one.

- Reproducer: `ball_box_shallow_overlap` in `auburn3d/tests/regressions.rs`, about 0.0395
  deep. The randomized `penetrates::*_gjk` tests are ignored for this item.
- It is not the iteration cap: still `None` at 1000 iterations. The simplex cycles through the
  same edge, triangle and tetrahedron states. One vertex, about (-0.045, 0.085, -0.300) in the
  Minkowski difference of `Ball(1.3853439)@(2.3622842,0.43820786,-1.2023413)` and
  `Box3d((1.8241154,1.6124979,1.7009325))@(-0.5612869,2.855741,0.73548675)` (b against a),
  stays in the simplex while the other vertices repeat.
- File: `auburn3d/src/algorithm/gjk.rs`. Suspects:
  - `handle_tetrahedron_simplex` takes the first face whose plane has the origin in front,
    which isn't always the origin's Voronoi region, and `handle_triangle_simplex` can then
    drop to an edge, throwing away the vertex that made progress;
  - no termination on lack of progress: a support point that gets no closer to the origin
    than the simplex already is should end the search instead of starting another loop.

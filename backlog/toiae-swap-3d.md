# 3D reversed `toiae`/`tttoiae` swap entry and exit

Reproduced on cf06cd2.

- `Point@0 .toiae(Ball(1)@(5,0,0), X)` returns (6,4); it should be (4,6).
- Ten `.map(|(exit, entry)| (entry, exit))` sites in
  `auburn3d/src/interactions/optimized/{vec3,transform3d}/{ball_point,ball_box,box_point}.rs`.
- The overlap times don't change when the call is reversed with −vel, so drop the swap.

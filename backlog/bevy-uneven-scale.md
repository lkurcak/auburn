# Bevy `Transform` with uneven scale

Reproduced on cf06cd2.

- 2D converts through `scale.max_element()` (`auburn2d/src/transformation/bevy_transform.rs`):
  - a (1,3,1)-scaled box "contains" (2.5,0);
  - z scale leaks into 2D ((1,1,5) returns true);
  - a mirrored triangle is missed.
- 3D is right since `Collider::support` maps directions through
  `Transformation3d::support_direction`, which bevy's `Transform` implements with its real
  per-axis scale. Its `inverse` still collapses the scale to the largest axis, but no 3D query
  calls it any more.
- Decide between proper support and rejecting it in 2D:
  - proper support: use the xy scale in `apply` and `support_direction` (the transpose of the
    linear part) instead of converting to `Transform2d`;
  - rejecting it: `debug_assert` uniform scale and use only xy.
- User-defined types with uneven scale get the right support points by implementing
  `support_direction`, as `Stretch` does in `auburn2d/tests/regressions.rs`.

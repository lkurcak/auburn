# Bevy `Transform` with uneven scale

Reproduced on cf06cd2.

- 2D converts through `scale.max_element()` (`auburn2d/src/transformation/bevy_transform.rs`):
  - a (1,3,1)-scaled box "contains" (2.5,0);
  - z scale leaks into 2D ((1,1,5) returns true);
  - a mirrored triangle is missed.
- 3D: `apply` uses the real per-axis scale, but `inverse` collapses it to the largest axis.
  With scale (−1,1,1), `Box3d(1)` support(+x) gives x=−1.
- Decide between proper support and rejecting it:
  - proper support: map the support direction through the transpose of the linear part
    instead of `inverse()` in `Collider::support` (`collider.rs`);
  - rejecting it: `debug_assert` uniform scale and use only xy in 2D.
- User-defined `Transformation2d` types with uneven scale also get wrong support points.
  Either document a uniform-scale contract or extend the trait.

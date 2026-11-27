pub mod transform3d;
pub mod vec3;

use crate::Vec3;
use crate::algorithm::shape_cast;
use crate::collider::Collider;
use crate::property::support::Support;
use crate::shape::Shape;
use crate::transformation::Transformation3d;

/// When `a`, moving at `vel`, overlaps `b`, over all of time, from their support functions:
/// for the pairs without an exact cast.
pub(crate) fn support_span<A, B, T>(
    a: &Collider<'_, A, T>,
    b: &Collider<'_, B, T>,
    vel: Vec3,
) -> Option<(f32, f32)>
where
    A: Shape + Support,
    B: Shape + Support,
    T: Transformation3d,
{
    shape_cast(|dir| a.support(dir), |dir| b.support(dir), vel)
}

/// `casts_from_span!(A, B, T, span)` implements the four casts of an `A` against a `B`, and of
/// a `B` against an `A`, both placed by a `T`, from `span(a, b, vel)`: when `a`, moving at
/// `vel`, overlaps `b`, over all of time. `casts_from_span!(A, T, span)` does a pair of `A`s.
macro_rules! casts_from_span {
    ($A:ty, $B:ty, $T:ty, $span:path) => {
        $crate::interactions::optimized::casts_from_span!(
            @each $A, $B, $T, |a, b, vel| $span(a, b, vel)
        );
        // `b` moving at `vel` is `a` moving at `-vel`.
        $crate::interactions::optimized::casts_from_span!(
            @each $B, $A, $T, |b, a, vel: $crate::Vec3| $span(a, b, -vel)
        );
    };
    ($A:ty, $T:ty, $span:path) => {
        $crate::interactions::optimized::casts_from_span!(
            @each $A, $A, $T, |a, b, vel| $span(a, b, vel)
        );
    };
    (@each $A:ty, $B:ty, $T:ty, $span:expr) => {
        $crate::interactions::optimized::casts_from_span!(
            @cast $A, $B, $T, $span,
            TimeOfImpact::toi -> f32,
            TimeOfImpactAndExit::toiae -> (f32, f32),
            TimeTravelingTimeOfImpact::tttoi -> f32,
            TimeTravelingTimeOfImpactAndExit::tttoiae -> (f32, f32)
        );
    };
    (@cast $A:ty, $B:ty, $T:ty, $span:expr, $($Trait:ident::$method:ident -> $Out:ty),*) => {$(
        impl $crate::relation::cast::$Trait<$crate::collider::Collider<'_, $B, $T>>
            for $crate::collider::Collider<'_, $A, $T>
        {
            fn $method(
                &self,
                other: &$crate::collider::Collider<'_, $B, $T>,
                vel: $crate::Vec3,
            ) -> Option<$Out> {
                let span: fn(
                    &$crate::collider::Collider<'_, $A, $T>,
                    &$crate::collider::Collider<'_, $B, $T>,
                    $crate::Vec3,
                ) -> Option<(f32, f32)> = $span;
                $crate::relation::cast::from_span::$method(span(self, other, vel))
            }
        }
    )*};
}
pub(crate) use casts_from_span;

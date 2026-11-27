//! Casts against convex polygons, by the separating axis theorem.
//!
//! Two convex shapes overlap while their spans overlap along every axis that could separate
//! them. For polygons those are the normals of their edges. X and Y are tested too, so that a
//! polygon with fewer than three vertices is the segment or point it looks like, not a line or
//! the whole plane. An axis that isn't needed never changes the answer.
//!
//! Nothing here depends on which way round the vertices go, or on how long the axes are, so
//! there is no tolerance to tune for the shapes' size.

use super::vec2::box_point::slab;
use crate::Vec2;
use crate::relation::cast::is_at_rest;

/// A normal to each edge of the polygon through `vertices`, not normalized.
pub(crate) fn edge_normals(vertices: &[Vec2]) -> impl Iterator<Item = Vec2> + '_ {
    let next = vertices.iter().cycle().skip(1);
    vertices.iter().zip(next).map(|(&p, &q)| (q - p).perp())
}

/// The span of `vertices` along `axis`.
pub(crate) fn project(vertices: &[Vec2], axis: Vec2) -> (f32, f32) {
    vertices
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            let d = p.dot(axis);
            (lo.min(d), hi.max(d))
        })
}

/// When a shape moving at `vel` overlaps another, over all of time (touching counts): from
/// when to when, or `None` if never.
///
/// `span_a` and `span_b` give each shape's span along an axis. `axes` has to hold the normals
/// of both shapes' edges; X and Y are added here.
pub(crate) fn swept_sat(
    span_a: impl Fn(Vec2) -> (f32, f32),
    span_b: impl Fn(Vec2) -> (f32, f32),
    axes: impl IntoIterator<Item = Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    // At rest is at rest along every axis, however long.
    let vel = if is_at_rest(vel) { Vec2::ZERO } else { vel };
    let (mut enter, mut exit) = (f32::NEG_INFINITY, f32::INFINITY);
    for axis in axes.into_iter().chain([Vec2::X, Vec2::Y]) {
        let (min_a, max_a) = span_a(axis);
        let (min_b, max_b) = span_b(axis);
        let (axis_enter, axis_exit) = slab(min_b - max_a, max_b - min_a, vel.dot(axis))?;
        enter = enter.max(axis_enter);
        exit = exit.min(axis_exit);
        if enter > exit {
            return None;
        }
    }
    Some((enter, exit))
}

/// When a point at `p`, moving at `vel`, is in the convex polygon through `vertices`, over all
/// of time (touching counts): from when to when, or `None` if never.
pub(crate) fn point_polygon(p: Vec2, vertices: &[Vec2], vel: Vec2) -> Option<(f32, f32)> {
    let span_p = |axis: Vec2| {
        let d = p.dot(axis);
        (d, d)
    };
    let span_poly = |axis| project(vertices, axis);
    swept_sat(span_p, span_poly, edge_normals(vertices), vel)
}

/// The four casts between `Collider<$A, $T>` and `Collider<$B, $T>`, both ways round, read off
/// `$span(a, b, vel)`: when `a`, moving at `vel`, overlaps `b`. Given one shape, the casts
/// between two of it.
macro_rules! casts_from_span {
    ($A:ty, $B:ty, $T:ty, $span:path) => {
        $crate::interactions::optimized::poly2d_utils::casts_from_span!(
            @each $A, $B, $T, |a, b, vel| $span(a, b, vel)
        );
        // `b` moving at `vel` is `a` moving at `-vel`.
        $crate::interactions::optimized::poly2d_utils::casts_from_span!(
            @each $B, $A, $T, |b, a, vel: $crate::Vec2| $span(a, b, -vel)
        );
    };
    ($A:ty, $T:ty, $span:path) => {
        $crate::interactions::optimized::poly2d_utils::casts_from_span!(
            @each $A, $A, $T, |a, b, vel| $span(a, b, vel)
        );
    };
    (@each $A:ty, $B:ty, $T:ty, $span:expr) => {
        $crate::interactions::optimized::poly2d_utils::casts_from_span!(
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
                vel: $crate::Vec2,
            ) -> Option<$Out> {
                let span: fn(
                    &$crate::collider::Collider<'_, $A, $T>,
                    &$crate::collider::Collider<'_, $B, $T>,
                    $crate::Vec2,
                ) -> Option<(f32, f32)> = $span;
                $crate::relation::cast::from_span::$method(span(self, other, vel))
            }
        }
    )*};
}
pub(crate) use casts_from_span;

//! The casts between any two colliders, whatever their shapes and transformations.
//!
//! Every cast reads its answer off [`span`]: when `a`, moving at `vel`, overlaps `b`, over all
//! of time. Built-in shapes that their transformations keep as they are (see
//! [`Shape::primitive`] and [`Transformation3d::rotation_and_scale`]) have exact formulas for
//! points against points, balls and boxes, and for balls against balls. A ball against
//! anything else goes through [`rounded_shape_cast`], as its centre rounded by its radius, and
//! every other pair through [`shape_cast`], on the colliders' support functions.
//!
//! The formulas are `#[inline]`: without it they aren't inlined into the crate that casts,
//! and the cheapest casts take twice as long.

use crate::algorithm::shape_cast;
use crate::algorithm::toi::rounded_shape_cast;
use crate::collider::Collider;
use crate::interactions::generic::{in_ball, in_cuboid};
use crate::property::support::Support;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
    direction_and_speed, from_span, is_at_rest,
};
use crate::shape::{Primitive, Shape};
use crate::transformation::Transformation3d;
use crate::{Quat, Vec3};

macro_rules! casts {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl<S1, S2, T1, T2> $Trait<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
        where
            S1: Shape + Support,
            S2: Shape + Support,
            T1: Transformation3d,
            T2: Transformation3d,
        {
            fn $method(&self, other: &Collider<'_, S2, T2>, vel: Vec3) -> Option<$Out> {
                from_span::$method(span(self, other, vel))
            }
        }
    )*};
}

casts! {
    TimeOfImpact::toi -> f32;
    TimeOfImpactAndExit::toiae -> (f32, f32);
    TimeTravelingTimeOfImpact::tttoi -> f32;
    TimeTravelingTimeOfImpactAndExit::tttoiae -> (f32, f32);
}

/// When `a`, moving at `vel`, overlaps `b`, over all of time (touching counts): from when to
/// when, or `None` if never.
fn span<S1, S2, T1, T2>(
    a: &Collider<'_, S1, T1>,
    b: &Collider<'_, S2, T2>,
    vel: Vec3,
) -> Option<(f32, f32)>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation3d,
    T2: Transformation3d,
{
    use Form::{Ball, Cuboid, Point};
    match (Form::of(a), Form::of(b)) {
        (Point(p), Point(q)) => point_point(p, q, vel),
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => point_ball(p - q, r, vel),
        (Ball(p, r), Ball(q, s)) => point_ball(p - q, r + s, vel),
        (Point(p), Cuboid(cuboid)) => point_cuboid(p, &cuboid, vel),
        // `b` moving at `-vel` is `a` moving at `vel`.
        (Cuboid(cuboid), Point(p)) => point_cuboid(p, &cuboid, -vel),
        (Ball(p, r), _) => rounded_shape_cast(|_| p, |dir| b.support(dir), r, vel),
        (_, Ball(q, r)) => rounded_shape_cast(|dir| a.support(dir), |_| q, r, vel),
        _ => shape_cast(|dir| a.support(dir), |dir| b.support(dir), vel),
    }
}

/// A collider in world space, in a form with exact casts if it has one.
pub(super) enum Form {
    Point(Vec3),
    /// A ball's centre and radius.
    Ball(Vec3, f32),
    Cuboid(Cuboid),
    /// Anything else, cast through its support function.
    Other,
}

/// A box in world space.
pub(super) struct Cuboid {
    pub(super) centre: Vec3,
    pub(super) rot: Quat,
    pub(super) half: Vec3,
}

impl Form {
    #[inline]
    pub(super) fn of<S: Shape, T: Transformation3d>(c: &Collider<'_, S, T>) -> Self {
        let at = c.isometry.apply_to_origin();
        let shape = c.shape.primitive();
        if let Primitive::Point = shape {
            return Form::Point(at);
        }
        let Some((rot, scale)) = c.isometry.rotation_and_scale() else {
            return Form::Other;
        };
        let s = scale.abs();
        match shape {
            // A ball stays a ball only if it is scaled the same along every axis.
            Primitive::Ball(ball) if s.x == s.y && s.y == s.z => Form::Ball(at, ball.radius * s.x),
            // Mirroring a box leaves it as it was.
            Primitive::Box3d(b) => Form::Cuboid(Cuboid {
                centre: at,
                rot,
                half: b.half_size * s,
            }),
            _ => Form::Other,
        }
    }
}

impl Cuboid {
    /// The point `p` in the box's frame: from its centre, turned with it.
    #[inline]
    pub(super) fn local(&self, p: Vec3) -> Vec3 {
        let p = p - self.centre;
        if self.rot == Quat::IDENTITY {
            p
        } else {
            self.rot.inverse() * p
        }
    }
}

/// When a point at `a` coincides with one at `b` while it moves at `vel`, over all of time: the
/// moment it passes through, `None` if it misses by more than rounding, or forever if it is at
/// rest on the other point.
#[inline]
fn point_point(a: Vec3, b: Vec3, vel: Vec3) -> Option<(f32, f32)> {
    let p = a - b;
    if is_at_rest(vel) {
        return (p == Vec3::ZERO).then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    let (dir, speed) = direction_and_speed(vel);
    let along = p.dot(dir);
    let miss = p - dir * along;
    // Positions are only good to a unit in their last place, so a path straight through can
    // seem to miss by a few of them.
    (miss.length() <= 4.0 * f32::EPSILON * (a.length() + b.length())).then(|| {
        let t = -along / speed;
        (t, t)
    })
}

/// When a point at `p`, relative to the centre of a ball of `radius`, is in the ball while it
/// moves at `vel`, over all of time (touching counts): from when to when, or `None` if never.
/// At rest, a point in the ball is in it forever.
#[inline]
fn point_ball(p: Vec3, radius: f32, vel: Vec3) -> Option<(f32, f32)> {
    if is_at_rest(vel) {
        return in_ball(p, radius).then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    // Along the path, in units of distance: the point passes closest to the centre at
    // `-along`, `miss` away from it, and is in the ball for a half chord either side.
    let (dir, speed) = direction_and_speed(vel);
    let along = p.dot(dir);
    let miss = p - dir * along;
    let half_chord_squared = radius * radius - miss.length_squared();
    if half_chord_squared < 0.0 {
        return None;
    }
    let half_chord = half_chord_squared.sqrt();
    Some(((-along - half_chord) / speed, (-along + half_chord) / speed))
}

/// When a point at `p`, moving at `vel`, is in `cuboid`, over all of time (touching counts):
/// from when to when, or `None` if never. That is where its path is between every pair of
/// opposite faces.
#[inline]
fn point_cuboid(p: Vec3, cuboid: &Cuboid, vel: Vec3) -> Option<(f32, f32)> {
    if is_at_rest(vel) {
        return in_cuboid(p, cuboid).then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    let offset = p - cuboid.centre;
    let axes = [Vec3::X, Vec3::Y, Vec3::Z].map(|axis| cuboid.rot * axis);
    let (mut entry, mut exit) = (f32::NEG_INFINITY, f32::INFINITY);
    for axis in axes {
        // The box's half span along `axis`, which works for axes of any length.
        let reach = axes
            .iter()
            .zip(cuboid.half.to_array())
            .map(|(&edge, half)| half * axis.dot(edge).abs())
            .sum::<f32>();
        let d = offset.dot(axis);
        let (axis_entry, axis_exit) = slab(-reach - d, reach - d, vel.dot(axis))?;
        entry = entry.max(axis_entry);
        exit = exit.min(axis_exit);
        if entry > exit {
            return None;
        }
    }
    Some((entry, exit))
}

/// When something moving at `vel` along one axis is within `lo..=hi` of where it starts: from
/// when to when (unbounded in both directions if it doesn't move), or `None` if never.
///
/// A velocity too small for its reciprocal to be finite (zero of either sign, or subnormal)
/// counts as not moving: the slab method would otherwise multiply 0 by infinity where the
/// motion starts exactly on `lo` or `hi`.
#[inline]
fn slab(lo: f32, hi: f32, vel: f32) -> Option<(f32, f32)> {
    let inv_vel = 1.0 / vel;
    if !inv_vel.is_finite() {
        return (lo <= 0.0 && 0.0 <= hi).then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    let (a, b) = (lo * inv_vel, hi * inv_vel);
    if a <= b { Some((a, b)) } else { Some((b, a)) }
}

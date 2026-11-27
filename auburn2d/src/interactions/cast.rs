//! The casts between any two colliders, whatever their shapes and transformations.
//!
//! Every cast reads its answer off [`span`]: when `a`, moving at `vel`, overlaps `b`, over all
//! of time. Built-in shapes that their transformations keep as they are (see
//! [`Shape::primitive`] and [`Transformation2d::rotation_and_scale`]) have exact formulas:
//! points, balls and boxes against each other, and polygons against points, boxes and
//! polygons. Every other pair goes through [`shape_cast`], on the colliders' support
//! functions: a ball against a polygon, and any shape without a primitive or under a
//! transformation without a rotation and scale.
//!
//! None of the formulas has a tolerance to tune for the shapes' size. Boxes turned the same
//! way, as boxes that don't rotate are, take the fewest steps.
//!
//! The formulas are `#[inline]`: without it they aren't inlined into the crate that casts,
//! and the cheapest casts take twice as long.

use crate::Vec2;
use crate::algorithm::shape_cast;
use crate::collider::Collider;
use crate::property::support::Support;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
    direction_and_speed, from_span, is_at_rest,
};
use crate::rotor2d::Rotor2d;
use crate::shape::{Primitive, Shape};
use crate::transformation::Transformation2d;
use std::borrow::Cow;

macro_rules! casts {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl<S1, S2, T1, T2> $Trait<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
        where
            S1: Shape + Support,
            S2: Shape + Support,
            T1: Transformation2d,
            T2: Transformation2d,
        {
            fn $method(&self, other: &Collider<'_, S2, T2>, vel: Vec2) -> Option<$Out> {
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
#[inline]
fn span<S1, S2, T1, T2>(
    a: &Collider<'_, S1, T1>,
    b: &Collider<'_, S2, T2>,
    vel: Vec2,
) -> Option<(f32, f32)>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    use Form::{Ball, Point, Polygon, Rect};
    match (Form::of(a), Form::of(b)) {
        (Point(p), Point(q)) => point_point(p, q, vel),
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => point_ball(p - q, r, vel),
        (Ball(p, r), Ball(q, s)) => point_ball(p - q, r + s, vel),
        (Ball(p, r), Rect(rect)) => ball_rect(p, r, &rect, vel),
        // `b` moving at `-vel` is `a` moving at `vel`.
        (Rect(rect), Ball(p, r)) => ball_rect(p, r, &rect, -vel),
        (Point(p), Rect(rect)) => rect_rect(&rect.point(p), &rect, vel),
        (Rect(rect), Point(p)) => rect_rect(&rect, &rect.point(p), vel),
        (Rect(r), Rect(s)) => rect_rect(&r, &s, vel),
        (x @ (Point(_) | Rect(_) | Polygon(_)), y @ (Point(_) | Rect(_) | Polygon(_))) => {
            separating_axes(&x, &y, vel)
        }
        _ => shape_cast(|dir| a.support(dir), |dir| b.support(dir), vel),
    }
}

/// A collider in world space, in a form with exact casts if it has one.
enum Form<'a> {
    Point(Vec2),
    /// A ball's centre and radius.
    Ball(Vec2, f32),
    Rect(Rect),
    Polygon(Polygon<'a>),
    /// Anything else, cast through its support function.
    Other,
}

/// A box in world space.
#[derive(Clone, Copy)]
struct Rect {
    centre: Vec2,
    rot: Rotor2d,
    /// Where `rot` turns X and Y, along the box's edges.
    axes: [Vec2; 2],
    half: Vec2,
}

/// A convex polygon in world space: its vertices, relative to `origin`.
struct Polygon<'a> {
    origin: Vec2,
    vertices: Cow<'a, [Vec2]>,
}

impl<'a> Form<'a> {
    #[inline]
    fn of<S: Shape, T: Transformation2d>(c: &Collider<'a, S, T>) -> Self {
        let at = c.isometry.apply_to_origin();
        let shape = c.shape.primitive();
        if let Primitive::Point = shape {
            return Form::Point(at);
        }
        let Some((rot, scale)) = c.isometry.rotation_and_scale() else {
            return Form::Other;
        };
        match shape {
            // A ball stays a ball only if it is scaled the same along both axes.
            Primitive::Ball(ball) if scale.x.abs() == scale.y.abs() => {
                Form::Ball(at, ball.radius * scale.x.abs())
            }
            // Mirroring a box leaves it as it was.
            Primitive::Box2d(b) => Form::Rect(Rect {
                centre: at,
                rot,
                axes: [rot * Vec2::X, rot * Vec2::Y],
                half: b.half_size * scale.abs(),
            }),
            Primitive::Poly2d(poly) => {
                let local = poly.vertices();
                let vertices = if rot == Rotor2d::IDENTITY && scale == Vec2::ONE {
                    Cow::Borrowed(local)
                } else {
                    Cow::Owned(local.iter().map(|&p| rot * (scale * p)).collect())
                };
                Form::Polygon(Polygon {
                    origin: at,
                    vertices,
                })
            }
            _ => Form::Other,
        }
    }

    /// The point that [`span`](Self::span) measures from, for a point, box or polygon.
    fn anchor(&self) -> Vec2 {
        match self {
            Form::Point(p) => *p,
            Form::Rect(rect) => rect.centre,
            Form::Polygon(polygon) => polygon.origin,
            Form::Ball(..) | Form::Other => unreachable!("not a point, box or polygon"),
        }
    }

    /// The span of a point, box or polygon along `axis`, measured from its anchor.
    fn span(&self, axis: Vec2) -> (f32, f32) {
        match self {
            Form::Point(_) => (0.0, 0.0),
            Form::Rect(rect) => {
                let [x, y] = rect.axes;
                let reach = rect.half.x * axis.dot(x).abs() + rect.half.y * axis.dot(y).abs();
                (-reach, reach)
            }
            Form::Polygon(polygon) => project(&polygon.vertices, axis),
            Form::Ball(..) | Form::Other => unreachable!("not a point, box or polygon"),
        }
    }
}

impl Rect {
    /// A point at `p`, as a box turned like this one.
    #[inline]
    fn point(&self, p: Vec2) -> Rect {
        Rect {
            centre: p,
            half: Vec2::ZERO,
            ..*self
        }
    }

    /// The point `p` and the velocity `vel` in the box's frame: from its centre, turned with
    /// it.
    #[inline]
    fn in_frame(&self, p: Vec2, vel: Vec2) -> (Vec2, Vec2) {
        let p = p - self.centre;
        if self.rot == Rotor2d::IDENTITY {
            return (p, vel);
        }
        // Turning a velocity at rest could make one of its components count as moving.
        let vel = if is_at_rest(vel) { Vec2::ZERO } else { vel };
        let inverse = self.rot.inverse();
        (inverse * p, inverse * vel)
    }
}

/// When a point at `a` coincides with one at `b` while it moves at `vel`, over all of time: the
/// moment it passes through, `None` if it misses by more than rounding, or forever if it is at
/// rest on the other point.
#[inline]
fn point_point(a: Vec2, b: Vec2, vel: Vec2) -> Option<(f32, f32)> {
    let p = a - b;
    if is_at_rest(vel) {
        return (p == Vec2::ZERO).then_some((f32::NEG_INFINITY, f32::INFINITY));
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
fn point_ball(p: Vec2, radius: f32, vel: Vec2) -> Option<(f32, f32)> {
    if is_at_rest(vel) {
        return (p.length_squared() <= radius * radius)
            .then_some((f32::NEG_INFINITY, f32::INFINITY));
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

/// When a point at `p`, moving at `vel`, is in the box at the origin with half size `half`,
/// over all of time (touching counts): from when to when, or `None` if never.
#[inline]
fn point_aabb(p: Vec2, half: Vec2, vel: Vec2) -> Option<(f32, f32)> {
    let (x_entry, x_exit) = slab(-half.x - p.x, half.x - p.x, vel.x)?;
    let (y_entry, y_exit) = slab(-half.y - p.y, half.y - p.y, vel.y)?;
    let (entry, exit) = (x_entry.max(y_entry), x_exit.min(y_exit));
    (entry <= exit).then_some((entry, exit))
}

/// When box `a`, moving at `vel`, overlaps box `b`, over all of time.
#[inline]
fn rect_rect(a: &Rect, b: &Rect, vel: Vec2) -> Option<(f32, f32)> {
    if a.rot != b.rot {
        return separating_axes(&Form::Rect(*a), &Form::Rect(*b), vel);
    }
    // Turned the same way, `a` overlaps `b` while its centre is in a box with both half sizes.
    let (p, vel) = b.in_frame(a.centre, vel);
    point_aabb(p, a.half + b.half, vel)
}

/// When a ball at `centre`, moving at `vel`, overlaps `rect`, over all of time.
///
/// Its centre is then within `radius` of the box. That rounded box is the box widened by
/// `radius`, except in the four squares at its corners, where only a quarter disc around the
/// box's corner is in it. A path that crosses such a square enters the rounded box through
/// that disc, or leaves the widened box again without touching it.
#[inline]
fn ball_rect(centre: Vec2, radius: f32, rect: &Rect, vel: Vec2) -> Option<(f32, f32)> {
    let half = rect.half;
    if is_at_rest(vel) {
        let (p, _) = rect.in_frame(centre, Vec2::ZERO);
        let outside = (p.abs() - half).max(Vec2::ZERO);
        let inside = outside.length_squared() <= radius * radius;
        return inside.then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    // In the box's frame, and in units of distance along the path.
    let (dir, speed) = direction_and_speed(vel);
    let (p, dir) = rect.in_frame(centre, dir);
    let (entry, exit) = point_aabb(p, half + radius, dir)?;
    // The corner disc that the path is in at distance `s`, if it is in a corner square.
    let disc = |s: f32| {
        let q = p + dir * s;
        let in_corner = q.x.abs() > half.x && q.y.abs() > half.y;
        in_corner.then(|| point_ball(p - half * q.signum(), radius, dir))
    };
    let entry = match disc(entry) {
        None => entry,
        Some(None) => return None,
        Some(Some((disc_entry, _))) => disc_entry,
    };
    // Rounding can make the other side miss a graze, which enters and exits at once.
    let exit = match disc(exit) {
        None => exit,
        Some(disc) => disc.map_or(entry, |(_, disc_exit)| disc_exit),
    };
    Some((entry / speed, exit.max(entry) / speed))
}

/// When `a`, moving at `vel`, overlaps `b`, over all of time, by the separating axis theorem.
/// Each is a point, box or polygon.
///
/// Two convex shapes overlap while their spans overlap along every axis that could separate
/// them: for these, the normals of their edges. Against a polygon, X and Y are tested too, so
/// that one with fewer than three vertices is the segment or point it looks like, not a line
/// or the whole plane. An axis that isn't needed never changes the answer, and nothing here
/// depends on which way round a polygon goes or on how long the axes are.
fn separating_axes(a: &Form<'_>, b: &Form<'_>, vel: Vec2) -> Option<(f32, f32)> {
    // At rest is at rest along every axis, however long.
    let vel = if is_at_rest(vel) { Vec2::ZERO } else { vel };
    // Measured from `b`'s anchor, so that the spans stay as precise as the shapes far from
    // the origin.
    let offset = a.anchor() - b.anchor();
    let (mut entry, mut exit) = (f32::NEG_INFINITY, f32::INFINITY);
    let mut along = |axis: Vec2| {
        let (min_a, max_a) = a.span(axis);
        let (min_b, max_b) = b.span(axis);
        let d = offset.dot(axis);
        let (axis_entry, axis_exit) = slab(min_b - max_a - d, max_b - min_a - d, vel.dot(axis))?;
        entry = entry.max(axis_entry);
        exit = exit.min(axis_exit);
        (entry <= exit).then_some(())
    };
    let mut x_and_y = false;
    for shape in [a, b] {
        match shape {
            Form::Rect(rect) if rect.rot == Rotor2d::IDENTITY => x_and_y = true,
            Form::Rect(rect) => {
                along(rect.axes[0])?;
                along(rect.axes[1])?;
            }
            Form::Polygon(polygon) => {
                x_and_y = true;
                for axis in edge_normals(&polygon.vertices) {
                    along(axis)?;
                }
            }
            _ => {}
        }
    }
    if x_and_y {
        along(Vec2::X)?;
        along(Vec2::Y)?;
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

/// A normal to each edge of the polygon through `vertices`, not normalized.
fn edge_normals(vertices: &[Vec2]) -> impl Iterator<Item = Vec2> + '_ {
    let next = vertices.iter().skip(1).chain(vertices.first());
    vertices.iter().zip(next).map(|(&p, &q)| (q - p).perp())
}

/// The span of `vertices` along `axis`.
fn project(vertices: &[Vec2], axis: Vec2) -> (f32, f32) {
    vertices
        .iter()
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            let d = p.dot(axis);
            (lo.min(d), hi.max(d))
        })
}

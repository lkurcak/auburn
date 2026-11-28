//! The casts between any two colliders, whatever their shapes and transformations.
//!
//! Built-in shapes that their transformations keep as they are (see [`Shape::primitive`] and
//! [`Transformation2d::rotation_and_scale`]) have exact formulas: points, balls and boxes
//! against each other, and polygons against points, boxes and polygons. Every other pair goes
//! through [`rounded_shape_cast`], on the colliders' support functions, with a ball as its
//! center rounded by its radius: a ball against a polygon, and any shape without a primitive
//! or under a transformation without a rotation and scale.
//!
//! [`TimeOfImpact::toi`] and [`Cast::cast`] look only forward, from now on, and only for the
//! first touch, as a game's own formula would: [`touch`] gives it, with the contact's normal,
//! which `toi` doesn't use and the compiler leaves out. The other casts read their answers off
//! [`span`]: when `a`, moving at `vel`, overlaps `b`, over all of time.
//!
//! If the shapes overlap already, [`Cast::cast`] parts them by the shortest move instead, as
//! [`Penetrates`](crate::relation::penetrates::Penetrates) does, in
//! [`generic`](super::generic). Either way, [`contact_point`] finds where they touch.
//!
//! None of the formulas has a tolerance to tune for the shapes' size. Boxes turned the same
//! way, as boxes that don't rotate are, take the fewest steps.
//!
//! The formulas are `#[inline]`: without it they aren't inlined into the crate that casts.
//! Those of `toi` and `cast` for the basic pairs, the matches on the pair's forms and the
//! forms themselves are `#[inline(always)]`, as LLVM keeps some of them as calls otherwise, in
//! a caller as large as `cast`. A call hides which pair it is, so every other pair's code
//! stays too, and the cheapest casts take three times as long. What only polygons and turned
//! boxes need is kept out of line.

use crate::Vec2;
use crate::algorithm::toi::{Radii, nearest_to_origin, rounded_shape_cast, sweep};
use crate::algorithm::{epa, gjk};
use crate::collider::Collider;
use crate::interactions::generic::{
    ball_in_rect, in_ball, in_rounded_aabb, length_and_direction, penetration,
};
use crate::property::support::Support;
use crate::relation::cast::{
    Cast, CastHit, TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact,
    TimeTravelingTimeOfImpactAndExit, against, direction_and_speed, from_span, is_at_rest, unit,
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
    TimeOfImpactAndExit::toiae -> (f32, f32);
    TimeTravelingTimeOfImpact::tttoi -> f32;
    TimeTravelingTimeOfImpactAndExit::tttoiae -> (f32, f32);
}

impl<S1, S2, T1, T2> TimeOfImpact<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    fn toi(&self, other: &Collider<'_, S2, T2>, vel: Vec2) -> Option<f32> {
        let (x, y) = (Form::of(self), Form::of(other));
        match touch(&x, &y, vel) {
            Some(touch) => touch.map(Touch::time),
            None => from_span::toi(support_span(self, other, &x, &y, vel)),
        }
    }
}

impl<S1, S2, T1, T2> Cast<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    fn cast(&self, other: &Collider<'_, S2, T2>, vel: Vec2) -> Option<CastHit> {
        hit(self, other, vel)
    }
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
    let (x, y) = (Form::of(a), Form::of(b));
    match exact(&x, &y, vel) {
        Some(span) => span.map(|span| (span.entry, span.exit)),
        None => support_span(a, b, &x, &y, vel),
    }
}

/// [`span`] through the support functions of the colliders' cores, `x` and `y` being their
/// forms.
fn support_span<S1, S2, T1, T2>(
    a: &Collider<'_, S1, T1>,
    b: &Collider<'_, S2, T2>,
    x: &Form<'_>,
    y: &Form<'_>,
    vel: Vec2,
) -> Option<(f32, f32)>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    let origin = y.origin(b);
    let (core_a, core_b) = (x.core(a, origin), y.core(b, origin));
    rounded_shape_cast(core_a, core_b, radii(x, y), vel)
}

/// When and how `a`, moving at `vel`, first touches `b` from now on: see [`Cast::cast`].
#[inline(always)]
fn hit<S1, S2, T1, T2>(
    a: &Collider<'_, S1, T1>,
    b: &Collider<'_, S2, T2>,
    vel: Vec2,
) -> Option<CastHit>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    let (x, y) = (Form::of(a), Form::of(b));
    let Some(touch) = touch(&x, &y, vel) else {
        let origin = y.origin(b);
        let (core_a, core_b) = (x.core(a, origin), y.core(b, origin));
        let hit = support_hit(core_a, core_b, radii(&x, &y), vel)?;
        return Some(CastHit {
            point: hit.point + origin,
            ..hit
        });
    };
    Some(match touch? {
        Touch::At { time, normal } => CastHit {
            time,
            normal,
            point: contact_point(&x, &y, time * vel, normal),
        },
        Touch::Overlapping => {
            let (normal, depth) = penetration(&x, &y, vel);
            CastHit {
                time: 0.0,
                normal,
                point: contact_point(&x, &y, depth * normal, normal),
            }
        }
    })
}

/// How `a`, moving at `vel`, first touches `b` from now on, by the pair's exact formula:
/// `None` if it has none, and `Some(None)` if they never touch from now on.
#[inline(always)]
fn touch(a: &Form<'_>, b: &Form<'_>, vel: Vec2) -> Option<Option<Touch>> {
    use Form::{Ball, Point, Polygon, Rect};
    Some(match (a, b) {
        (Point(p), Point(q)) => Touch::from_span(point_point(*p, *q, vel), vel),
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => point_ball_touch(*p - *q, *r, vel),
        (Ball(p, r), Ball(q, s)) => point_ball_touch(*p - *q, r + s, vel),
        (Ball(p, r), Rect(rect)) => ball_rect_touch(*p, *r, rect, vel),
        // `b` moving at `-vel` is `a` moving at `vel`.
        (Rect(rect), Ball(p, r)) => ball_rect_touch(*p, *r, rect, -vel).map(Touch::swapped),
        (Point(p), Rect(rect)) => aligned_touch(*p, rect.half, rect, vel),
        (Rect(rect), Point(p)) => aligned_touch(*p, rect.half, rect, -vel).map(Touch::swapped),
        (Rect(r), Rect(s)) => rect_rect_touch(r, s, vel),
        (Point(_) | Rect(_) | Polygon(_), Point(_) | Rect(_) | Polygon(_)) => {
            Touch::from_span(separating_axes(a, b, vel), vel)
        }
        _ => return None,
    })
}

/// When `a`, moving at `vel`, overlaps `b`, over all of time, by the pair's exact formula:
/// `None` if it has none, and `Some(None)` if they never overlap.
#[inline]
fn exact(a: &Form<'_>, b: &Form<'_>, vel: Vec2) -> Option<Option<Span>> {
    use Form::{Ball, Point, Polygon, Rect};
    Some(match (a, b) {
        (Point(p), Point(q)) => point_point(*p, *q, vel),
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => point_ball(*p - *q, *r, vel),
        (Ball(p, r), Ball(q, s)) => point_ball(*p - *q, r + s, vel),
        (Ball(p, r), Rect(rect)) => ball_rect(*p, *r, rect, vel),
        // `b` moving at `-vel` is `a` moving at `vel`.
        (Rect(rect), Ball(p, r)) => ball_rect(*p, *r, rect, -vel).map(Span::swapped),
        (Point(p), Rect(rect)) => rect_rect(&rect.point(*p), rect, vel),
        (Rect(rect), Point(p)) => rect_rect(rect, &rect.point(*p), vel),
        (Rect(r), Rect(s)) => rect_rect(r, s, vel),
        (Point(_) | Rect(_) | Polygon(_), Point(_) | Rect(_) | Polygon(_)) => {
            separating_axes(a, b, vel)
        }
        _ => return None,
    })
}

/// A point where `a`, moved by `shift`, touches `b`, with `normal` the contact's normal. Only
/// for pairs with an exact formula.
#[inline(always)]
fn contact_point(a: &Form<'_>, b: &Form<'_>, shift: Vec2, normal: Vec2) -> Vec2 {
    use Form::{Ball, Point, Rect};
    match (a, b) {
        (_, Point(q)) => *q,
        (Point(p), _) => *p + shift,
        (_, Ball(q, r)) => *q + normal * *r,
        (Ball(p, r), _) => *p + shift - normal * *r,
        (Rect(r), Rect(s)) => rect_contact_point(r, s, shift, normal),
        _ => facing_point(a, b, shift, normal),
    }
}

/// [`contact_point`] for two boxes.
#[inline(always)]
fn rect_contact_point(a: &Rect, b: &Rect, shift: Vec2, normal: Vec2) -> Vec2 {
    if a.rot != b.rot {
        return facing_point(&Form::Rect(*a), &Form::Rect(*b), shift, normal);
    }
    // In `b`'s frame, the middle of where they overlap: on its side, as they touch.
    let center = b.local(a.centre + shift);
    let lo = (center - a.half).max(-b.half);
    let hi = (center + a.half).min(b.half);
    b.centre + b.in_world(0.5 * (lo + hi))
}

/// [`contact_point`] for boxes turned different ways and polygons: where the side or corner
/// of each that faces the other overlap, along the contact, the middle of it, on `b`'s side.
fn facing_point(a: &Form<'_>, b: &Form<'_>, shift: Vec2, normal: Vec2) -> Vec2 {
    let across = normal.perp();
    let offset = (a.anchor() - b.anchor() + shift).dot(across);
    let (b_lo, b_hi, level) = b.facing(normal, across);
    let (a_lo, a_hi, _) = a.facing(-normal, across);
    let (lo, hi) = (b_lo.max(a_lo + offset), b_hi.min(a_hi + offset));
    b.anchor() + normal * level + across * (0.5 * (lo + hi))
}

/// The radii that round the cores of `a` and `b` in a cast through support functions: a
/// ball's, and none for any other shape.
fn radii(a: &Form<'_>, b: &Form<'_>) -> Radii {
    let radius = |form: &Form<'_>| match form {
        Form::Ball(_, r) => *r,
        _ => 0.0,
    };
    Radii {
        a: radius(a),
        b: radius(b),
    }
}

/// [`hit`], through the support functions of shapes' cores, rounded by `radii`, with the point
/// in the frame they are in.
fn support_hit<F1, F2>(core_a: F1, core_b: F2, radii: Radii, vel: Vec2) -> Option<CastHit>
where
    F1: Fn(Vec2) -> Vec2 + Copy,
    F2: Fn(Vec2) -> Vec2 + Copy,
{
    if is_at_rest(vel) {
        // At rest, only shapes that overlap give a span.
        rounded_shape_cast(core_a, core_b, radii, vel)?;
    } else {
        // As `rounded_shape_cast` has it, so that the time is `toi`'s.
        let (dir, speed) = direction_and_speed(vel);
        let (entry, exit) = sweep(core_a, core_b, radii, dir)?;
        let (entry_time, exit_time) = (entry.t / speed, exit.t / speed);
        if exit_time < 0.0 {
            return None;
        }
        if entry_time >= 0.0 {
            return Some(CastHit {
                time: entry_time.max(0.0),
                normal: entry.normal,
                point: entry.point,
            });
        }
    }
    let normal = way_out(core_a, core_b, radii).unwrap_or_else(|| against(vel));
    // Where `a`, moving out along `normal`, stops overlapping `b`.
    let point = sweep(core_a, core_b, radii, normal)
        .map_or_else(|| core_b(normal) + normal * radii.b, |(_, exit)| exit.point);
    Some(CastHit {
        time: 0.0,
        normal,
        point,
    })
}

/// The shortest way out for `a`, which overlaps `b` now: the unit normal out of `b`. `None` if
/// it can't be found, which happens only for shapes that touch.
fn way_out<F1, F2>(core_a: F1, core_b: F2, radii: Radii) -> Option<Vec2>
where
    F1: Fn(Vec2) -> Vec2 + Copy,
    F2: Fn(Vec2) -> Vec2 + Copy,
{
    // If the cores don't overlap, out of the rounding, straight away from their nearest points.
    if let Some((near, _)) = nearest_to_origin(core_a, core_b) {
        return unit(-near);
    }
    // Otherwise EPA has the way out of the cores, unless they only touch and GJK and EPA find
    // no direction.
    const GJK_MAX_ITERATIONS: usize = 30;
    gjk(core_a, core_b, GJK_MAX_ITERATIONS)
        .and_then(|simplex| epa::penetration(simplex, core_a, core_b))
        // EPA's normal pushes `b` away from `a`.
        .map(|(normal, _)| -normal)
        .or_else(|| across_middles(core_a, core_b, radii))
}

/// The normal of shapes that touch, where GJK and EPA find none: where `a`, moving away from
/// `b` along the line through their cores' middles, stops overlapping it. Any line from a
/// point inside `b` to one inside `a` crosses where they touch.
fn across_middles<F1, F2>(core_a: F1, core_b: F2, radii: Radii) -> Option<Vec2>
where
    F1: Fn(Vec2) -> Vec2 + Copy,
    F2: Fn(Vec2) -> Vec2 + Copy,
{
    let middle = |support: &dyn Fn(Vec2) -> Vec2| {
        let sides = [Vec2::X, Vec2::NEG_X, Vec2::Y, Vec2::NEG_Y];
        sides.iter().map(|&dir| support(dir)).sum::<Vec2>() / 4.0
    };
    let dir = unit(middle(&core_a) - middle(&core_b))?;
    let (_, exit) = sweep(core_a, core_b, radii, dir)?;
    Some(exit.normal)
}

/// How `a`, moving at `vel`, first touches `b` from now on.
#[derive(Clone, Copy)]
enum Touch {
    /// They overlap already, touching counts: at time 0.
    Overlapping,
    /// They first touch at `time`, from now on (`time >= 0`), with `normal` the contact's
    /// normal, a unit vector out of `b`.
    At { time: f32, normal: Vec2 },
}

impl Touch {
    /// When they first touch, from now on.
    fn time(self) -> f32 {
        match self {
            Touch::Overlapping => 0.0,
            Touch::At { time, .. } => time,
        }
    }

    /// How `b`, moving at `-vel`, first touches `a`, from how `a`, moving at `vel`, first
    /// touches `b`: at the same time, with the normal out of the other shape.
    fn swapped(self) -> Self {
        match self {
            Touch::Overlapping => Touch::Overlapping,
            Touch::At { time, normal } => Touch::At {
                time,
                normal: -normal,
            },
        }
    }

    /// The same touch, with its normal from `rect`'s frame turned back into the world.
    fn in_world(self, rect: &Rect) -> Self {
        match self {
            Touch::Overlapping => Touch::Overlapping,
            Touch::At { time, normal } => Touch::At {
                time,
                normal: rect.in_world(normal),
            },
        }
    }

    /// The first touch from now on, from the span of a cast at `vel` over all of time.
    fn from_span(span: Option<Span>, vel: Vec2) -> Option<Self> {
        let span = span.filter(|span| span.exit >= 0.0)?;
        Some(if span.entry >= 0.0 {
            Touch::At {
                time: span.entry,
                normal: unit(span.normal).unwrap_or_else(|| against(vel)),
            }
        } else {
            Touch::Overlapping
        })
    }
}

/// When `a`, moving at `vel`, overlaps `b`, over all of time, and how they touch as it starts.
#[derive(Clone, Copy)]
struct Span {
    entry: f32,
    exit: f32,
    /// Along the contact's normal at `entry`, out of `b`, not necessarily of length 1.
    normal: Vec2,
}

impl Span {
    /// At rest, overlapping for all of time: there is no entry, so no normal.
    const ALWAYS: Self = Self {
        entry: f32::NEG_INFINITY,
        exit: f32::INFINITY,
        normal: Vec2::ZERO,
    };

    /// The span of `b` moving at `-vel` against `a`, from that of `a` moving at `vel`: the
    /// same times, with the normal out of the other shape.
    fn swapped(self) -> Self {
        Self {
            normal: -self.normal,
            ..self
        }
    }
}

/// A collider in world space, in a form with exact casts if it has one.
pub(super) enum Form<'a> {
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
pub(super) struct Rect {
    pub(super) centre: Vec2,
    pub(super) rot: Rotor2d,
    /// Where `rot` turns X and Y, along the box's edges.
    axes: [Vec2; 2],
    pub(super) half: Vec2,
}

/// A convex polygon in world space: its vertices, relative to `origin`.
pub(super) struct Polygon<'a> {
    origin: Vec2,
    vertices: Cow<'a, [Vec2]>,
}

impl<'a> Form<'a> {
    #[inline(always)]
    pub(super) fn of<S: Shape, T: Transformation2d>(c: &Collider<'a, S, T>) -> Self {
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

    /// The point that [`span`](Self::span) measures from, for a point, ball, box or polygon.
    pub(super) fn anchor(&self) -> Vec2 {
        match self {
            Form::Point(p) | Form::Ball(p, _) => *p,
            Form::Rect(rect) => rect.centre,
            Form::Polygon(polygon) => polygon.origin,
            Form::Other => unreachable!("not a point, ball, box or polygon"),
        }
    }

    /// The span of a point, ball, box or polygon along `axis`, measured from its anchor.
    fn span(&self, axis: Vec2) -> (f32, f32) {
        match self {
            Form::Point(_) => (0.0, 0.0),
            Form::Ball(_, r) => {
                let reach = r * axis.length();
                (-reach, reach)
            }
            Form::Rect(rect) => {
                let [x, y] = rect.axes;
                let reach = rect.half.x * axis.dot(x).abs() + rect.half.y * axis.dot(y).abs();
                (-reach, reach)
            }
            Form::Polygon(polygon) => project(&polygon.vertices, axis),
            Form::Other => unreachable!("not a point, ball, box or polygon"),
        }
    }

    /// Where its core is measured from in a cast through support functions, as `b`: its anchor,
    /// a ball's centre, or where `collider` puts its origin.
    fn origin<S: Shape + Support, T: Transformation2d>(
        &self,
        collider: &Collider<'_, S, T>,
    ) -> Vec2 {
        match self {
            Form::Point(p) | Form::Ball(p, _) => *p,
            Form::Rect(rect) => rect.centre,
            Form::Polygon(polygon) => polygon.origin,
            Form::Other => collider.isometry.apply_to_origin(),
        }
    }

    /// The support function of its core, measured from `origin`: a ball is its centre, rounded
    /// by its radius, and any other shape is its own core.
    ///
    /// Shapes with a form are measured from their own anchor first, so that they are as precise
    /// far from the origin as near it. Any other shape goes through `collider`'s support
    /// function, in world space.
    fn core<'s, S: Shape + Support, T: Transformation2d>(
        &'s self,
        collider: &'s Collider<'_, S, T>,
        origin: Vec2,
    ) -> impl Fn(Vec2) -> Vec2 + Copy + 's {
        move |dir: Vec2| match self {
            Form::Point(p) | Form::Ball(p, _) => *p - origin,
            Form::Rect(rect) => {
                let [x, y] = rect.axes;
                let x = x * rect.half.x.copysign(dir.dot(x));
                let y = y * rect.half.y.copysign(dir.dot(y));
                rect.centre - origin + x + y
            }
            Form::Polygon(polygon) => polygon.origin - origin + furthest(&polygon.vertices, dir),
            Form::Other => collider.support(dir) - origin,
        }
    }

    /// The side or corner of a point, box or polygon that faces the unit vector `m`, measured
    /// from its anchor: where it spans along `across`, and how far along `m` it is.
    fn facing(&self, m: Vec2, across: Vec2) -> (f32, f32, f32) {
        match self {
            Form::Point(_) => (0.0, 0.0, 0.0),
            Form::Rect(rect) => {
                let [x, y] = [rect.axes[0] * rect.half.x, rect.axes[1] * rect.half.y];
                facing(&[x + y, y - x, -x - y, x - y], m, across)
            }
            Form::Polygon(polygon) => facing(&polygon.vertices, m, across),
            Form::Ball(..) | Form::Other => unreachable!("not a point, box or polygon"),
        }
    }
}

impl Rect {
    /// A point at `p`, as a box turned like this one.
    #[inline(always)]
    pub(super) fn point(&self, p: Vec2) -> Rect {
        Rect {
            centre: p,
            half: Vec2::ZERO,
            ..*self
        }
    }

    /// The point `p` in the box's frame: from its centre, turned with it.
    #[inline(always)]
    pub(super) fn local(&self, p: Vec2) -> Vec2 {
        let p = p - self.centre;
        if self.rot == Rotor2d::IDENTITY {
            p
        } else {
            self.rot.inverse() * p
        }
    }

    /// A vector `v` in the box's frame, turned back into the world.
    #[inline(always)]
    pub(super) fn in_world(&self, v: Vec2) -> Vec2 {
        if self.rot == Rotor2d::IDENTITY {
            v
        } else {
            self.rot * v
        }
    }

    /// The point `p` and the velocity `vel` in the box's frame: from its centre, turned with
    /// it.
    #[inline(always)]
    fn in_frame(&self, p: Vec2, vel: Vec2) -> (Vec2, Vec2) {
        let p = self.local(p);
        if self.rot == Rotor2d::IDENTITY {
            return (p, vel);
        }
        // Turning a velocity at rest could make one of its components count as moving.
        let vel = if is_at_rest(vel) { Vec2::ZERO } else { vel };
        (p, self.rot.inverse() * vel)
    }
}

/// When a point at `a` coincides with one at `b` while it moves at `vel`, over all of time: the
/// moment it passes through, `None` if it misses by more than rounding, or forever if it is at
/// rest on the other point.
#[inline]
fn point_point(a: Vec2, b: Vec2, vel: Vec2) -> Option<Span> {
    let p = a - b;
    if is_at_rest(vel) {
        return (p == Vec2::ZERO).then_some(Span::ALWAYS);
    }
    let (dir, speed) = direction_and_speed(vel);
    let along = p.dot(dir);
    let miss = p - dir * along;
    // Positions are only good to a unit in their last place, so a path straight through can
    // seem to miss by a few of them.
    (miss.length() <= 4.0 * f32::EPSILON * (a.length() + b.length())).then(|| {
        let t = -along / speed;
        Span {
            entry: t,
            exit: t,
            normal: -dir,
        }
    })
}

/// When a point at `p`, relative to the centre of a ball of `radius`, is in the ball while it
/// moves at `vel`, over all of time (touching counts): from when to when, or `None` if never.
/// At rest, a point in the ball is in it forever.
#[inline]
fn point_ball(p: Vec2, radius: f32, vel: Vec2) -> Option<Span> {
    if is_at_rest(vel) {
        return in_ball(p, radius).then_some(Span::ALWAYS);
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
    Some(Span {
        entry: (-along - half_chord) / speed,
        exit: (-along + half_chord) / speed,
        // Where the point enters, out from the centre.
        normal: miss - dir * half_chord,
    })
}

/// How a point at `p`, relative to the center of a ball of `radius`, first touches it from now
/// on, while it moves at `vel`.
///
/// It is in the ball while `|p + vel t|² <= radius²`, and the smaller root of that quadratic
/// is when it enters.
#[inline(always)]
fn point_ball_touch(p: Vec2, radius: f32, vel: Vec2) -> Option<Touch> {
    let c = p.length_squared() - radius * radius;
    if c <= 0.0 {
        return Some(Touch::Overlapping);
    }
    let time = point_ball_entry(p, c, radius, vel)?;
    // Where it enters, out from the center.
    let normal = match length_and_direction(p + vel * time) {
        Some((_, normal)) => normal,
        None => against(vel),
    };
    Some(Touch::At { time, normal })
}

/// When a point at `p` outside a ball of `radius` enters it, moving at `vel`, from now on:
/// [`point_ball_touch`], with `c` the square of its distance from the center less the
/// radius's.
#[inline(always)]
fn point_ball_entry(p: Vec2, c: f32, radius: f32, vel: Vec2) -> Option<f32> {
    let (speed_squared, entry) = chord_entry(p, c, radius, vel);
    if !speed_squared.is_normal() {
        return point_ball_entry_scaled(p, c, radius, vel);
    }
    entry
}

/// [`point_ball_entry`] at a speed whose square isn't a normal number: too slow or too fast,
/// so in units of its largest component, or at rest.
#[cold]
#[inline(never)]
fn point_ball_entry_scaled(p: Vec2, c: f32, radius: f32, vel: Vec2) -> Option<f32> {
    if is_at_rest(vel) {
        return None;
    }
    let largest = vel.abs().max_element();
    let (_, entry) = chord_entry(p, c, radius, vel / largest);
    Some(entry? / largest)
}

/// [`point_ball_entry`], right if the square of the speed, which it gives too, is a normal
/// number.
#[inline(always)]
fn chord_entry(p: Vec2, c: f32, radius: f32, vel: Vec2) -> (f32, Option<f32>) {
    let along = p.dot(vel);
    let speed_squared = vel.length_squared();
    // The square of the half chord, times `speed_squared`. The distance by which the path
    // misses the center is from the cross product, which stays precise however far away the
    // point starts.
    let miss = p.perp_dot(vel);
    let half_chord_squared = speed_squared * (radius * radius) - miss * miss;
    // The smaller root, in the form that doesn't cancel.
    let time = c / (half_chord_squared.sqrt() - along);
    // Moving away from the center, or across, it never gets nearer.
    let enters = along < 0.0 && half_chord_squared >= 0.0;
    (speed_squared, enters.then_some(time))
}

/// When a point at `p`, moving at `vel`, first touches the box at the origin with half size
/// `half`, from now on (touching counts): the time, 0 if it is in the box already, and the
/// normal out of the side it enters through then. `None` if never.
///
/// It is the slab test, read from now on: the point is in the box while it is between the
/// two sides of each axis, and it reaches first the side it moves towards. Along an axis it
/// doesn't move along, the reciprocal of its velocity is infinite, and so is the time to each
/// side, but where it starts on a side, 0 × ∞ is NaN, which `max` and `min` skip: it touches
/// that side all along. Which side is which goes by the velocity's sign, zeros too, with no
/// branch: random signs would mispredict one.
#[inline(always)]
fn point_aabb_entry(p: Vec2, half: Vec2, vel: Vec2) -> Option<(f32, Vec2)> {
    let inv = vel.recip();
    let side = half.copysign(inv);
    let (near, far) = ((-side - p) * inv, (side - p) * inv);
    let last = near.x.max(near.y);
    let entry = last.max(0.0);
    // At rest outside the box, every time is infinite: `f32::MAX` keeps the entry from fitting
    // before the exit.
    let exit = f32::MAX.min(far.x).min(far.y);
    if entry > exit {
        return None;
    }
    // Out of the side it enters through, against its velocity.
    let out = -Vec2::ONE.copysign(inv);
    let normal = if near.x == last {
        Vec2::new(out.x, 0.0)
    } else {
        Vec2::new(0.0, out.y)
    };
    Some((entry, normal))
}

/// How box `a`, moving at `vel`, first touches box `b` from now on.
#[inline(always)]
fn rect_rect_touch(a: &Rect, b: &Rect, vel: Vec2) -> Option<Touch> {
    if a.rot != b.rot {
        return turned_rect_rect_touch(a, b, vel);
    }
    // Turned the same way, `a` overlaps `b` while its center is in a box with both half sizes.
    aligned_touch(a.centre, a.half + b.half, b, vel)
}

/// [`rect_rect_touch`] for boxes turned different ways, by the separating axis theorem.
fn turned_rect_rect_touch(a: &Rect, b: &Rect, vel: Vec2) -> Option<Touch> {
    Touch::from_span(separating_axes(&Form::Rect(*a), &Form::Rect(*b), vel), vel)
}

/// How a point at `p`, moving at `vel`, first touches a box turned as `rect` is, at its center,
/// with half size `half`, from now on.
#[inline(always)]
fn aligned_touch(p: Vec2, half: Vec2, rect: &Rect, vel: Vec2) -> Option<Touch> {
    let (p, vel) = rect.in_frame(p, vel);
    let (time, normal) = point_aabb_entry(p, half, vel)?;
    Some(if time == 0.0 {
        Touch::Overlapping
    } else {
        Touch::At {
            time,
            normal: rect.in_world(normal),
        }
    })
}

/// How a ball at `center`, moving at `vel`, first touches `rect` from now on.
///
/// Its center then enters the rounded box of [`ball_rect`]. It enters the box widened by
/// `radius` first, and if that is in a corner square, it enters the rounded box through the
/// corner's disc, or not at all.
#[inline(always)]
fn ball_rect_touch(center: Vec2, radius: f32, rect: &Rect, vel: Vec2) -> Option<Touch> {
    let half = rect.half;
    let (p, vel) = rect.in_frame(center, vel);
    // As `Collides` has it, so that casts at rest agree with it.
    if in_rounded_aabb(p, half, radius) {
        return Some(Touch::Overlapping);
    }
    let (time, normal) = point_aabb_entry(p, half + radius, vel)?;
    let q = p + vel * time;
    if q.x.abs() > half.x && q.y.abs() > half.y {
        let touch = point_ball_touch(p - half.copysign(q), radius, vel)?;
        return Some(touch.in_world(rect));
    }
    // Along a side, it is in the widened box at time 0 only by rounding: at rest, apart, as
    // `Collides` finds it.
    if time == 0.0 && is_at_rest(vel) {
        return None;
    }
    Some(Touch::At {
        time,
        normal: rect.in_world(normal),
    })
}

/// When a point at `p`, moving at `vel`, is in the box at the origin with half size `half`,
/// over all of time (touching counts): from when to when, or `None` if never.
#[inline]
fn point_aabb(p: Vec2, half: Vec2, vel: Vec2) -> Option<Span> {
    let (x_entry, x_exit) = slab(-half.x - p.x, half.x - p.x, vel.x)?;
    let (y_entry, y_exit) = slab(-half.y - p.y, half.y - p.y, vel.y)?;
    let (entry, exit) = (x_entry.max(y_entry), x_exit.min(y_exit));
    // The point enters through the side of the slab it enters last that it moves away from.
    let normal = if x_entry >= y_entry {
        Vec2::new(-vel.x.signum(), 0.0)
    } else {
        Vec2::new(0.0, -vel.y.signum())
    };
    (entry <= exit).then_some(Span {
        entry,
        exit,
        normal,
    })
}

/// When box `a`, moving at `vel`, overlaps box `b`, over all of time.
#[inline]
fn rect_rect(a: &Rect, b: &Rect, vel: Vec2) -> Option<Span> {
    if a.rot != b.rot {
        return separating_axes(&Form::Rect(*a), &Form::Rect(*b), vel);
    }
    // Turned the same way, `a` overlaps `b` while its center is in a box with both half sizes.
    let (p, vel) = b.in_frame(a.centre, vel);
    let span = point_aabb(p, a.half + b.half, vel)?;
    Some(Span {
        normal: b.rot * span.normal,
        ..span
    })
}

/// When a ball at `centre`, moving at `vel`, overlaps `rect`, over all of time.
///
/// Its centre is then within `radius` of the box. That rounded box is the box widened by
/// `radius`, except in the four squares at its corners, where only a quarter disc around the
/// box's corner is in it. A path that crosses such a square enters the rounded box through
/// that disc, or leaves the widened box again without touching it.
#[inline]
fn ball_rect(centre: Vec2, radius: f32, rect: &Rect, vel: Vec2) -> Option<Span> {
    if is_at_rest(vel) {
        return ball_in_rect(centre, radius, rect).then_some(Span::ALWAYS);
    }
    let half = rect.half;
    // In the box's frame, and in units of distance along the path.
    let (dir, speed) = direction_and_speed(vel);
    let (p, dir) = rect.in_frame(centre, dir);
    let widened = point_aabb(p, half + radius, dir)?;
    let (entry, exit) = (widened.entry, widened.exit);
    // The corner disc that the path is in at distance `s`, if it is in a corner square.
    let disc = |s: f32| {
        let q = p + dir * s;
        let in_corner = q.x.abs() > half.x && q.y.abs() > half.y;
        in_corner.then(|| point_ball(p - half * q.signum(), radius, dir))
    };
    let entry = match disc(entry) {
        None => entry,
        Some(None) => return None,
        Some(Some(disc)) => disc.entry,
    };
    // Rounding can make the other side miss a graze, which enters and exits at once.
    let exit = match disc(exit) {
        None => exit,
        Some(disc) => disc.map_or(entry, |disc| disc.exit),
    };
    // Where the centre enters, out from the nearest point of the box.
    let q = p + dir * entry;
    let normal = unit(q - q.max(-half).min(half)).unwrap_or(widened.normal);
    Some(Span {
        entry: entry / speed,
        exit: exit.max(entry) / speed,
        normal: rect.rot * normal,
    })
}

/// When `a`, moving at `vel`, overlaps `b`, over all of time, by the separating axis theorem.
/// Each is a point, box or polygon.
///
/// Two convex shapes overlap while their spans overlap along every axis that could separate
/// them: for these, the normals of their edges. Against a polygon, X and Y are tested too, so
/// that one with fewer than three vertices is the segment or point it looks like, not a line
/// or the whole plane. An axis that isn't needed never changes the answer, and nothing here
/// depends on which way round a polygon goes or on how long the axes are.
fn separating_axes(a: &Form<'_>, b: &Form<'_>, vel: Vec2) -> Option<Span> {
    // At rest is at rest along every axis, however long.
    let vel = if is_at_rest(vel) { Vec2::ZERO } else { vel };
    // Measured from `b`'s anchor, so that the spans stay as precise as the shapes far from
    // the origin.
    let offset = a.anchor() - b.anchor();
    let mut span = Span::ALWAYS;
    each_axis(a, b, |axis| {
        let (lo, hi) = overlap(a, b, offset, axis);
        let speed = vel.dot(axis);
        let (axis_entry, axis_exit) = slab(lo, hi, speed)?;
        if axis_entry > span.entry {
            span.entry = axis_entry;
            // `a` enters through the side of the slab it moves away from.
            span.normal = if speed > 0.0 { -axis } else { axis };
        }
        span.exit = span.exit.min(axis_exit);
        (span.entry <= span.exit).then_some(())
    })?;
    Some(span)
}

/// The axes that could separate `a` from `b`, each a point, box or polygon, or a ball against a
/// polygon, handed to `along` until it gives `None`, which it then gives too.
///
/// For these, they are the normals of their edges. Against a polygon, X and Y are tested too,
/// so that one with fewer than three vertices is the segment or point it looks like, not a
/// line or the whole plane. A ball has no edges, but a polygon it doesn't overlap is apart
/// from it across an edge or along the line from its centre to the polygon's nearest vertex.
///
/// That holds only where they are, not along a cast, as a ball moving past a polygon could
/// first touch it at any vertex: Ball–Poly2d casts go through support functions.
#[inline]
pub(super) fn each_axis(
    a: &Form<'_>,
    b: &Form<'_>,
    mut along: impl FnMut(Vec2) -> Option<()>,
) -> Option<()> {
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
    if let (Form::Ball(centre, _), Form::Polygon(polygon))
    | (Form::Polygon(polygon), Form::Ball(centre, _)) = (a, b)
    {
        let centre = *centre - polygon.origin;
        along(nearest_vertex(&polygon.vertices, centre) - centre)?;
    }
    Some(())
}

/// How far along `axis`, times its length, `a` can move from where it is and still overlap
/// `b`: from `lo` to `hi`. `offset` is from `b`'s anchor to `a`'s.
#[inline]
pub(super) fn overlap(a: &Form<'_>, b: &Form<'_>, offset: Vec2, axis: Vec2) -> (f32, f32) {
    let (min_a, max_a) = a.span(axis);
    let (min_b, max_b) = b.span(axis);
    let d = offset.dot(axis);
    (min_b - max_a - d, max_b - min_a - d)
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

/// The side or corner of the polygon through `vertices` that faces the unit vector `m`: where
/// it spans along `across`, and how far along `m` it is.
///
/// A vertex within rounding of the furthest along `m` is part of it, so that a side square on
/// to `m` counts whole. Any point of the side or corner, as found, is within that of the
/// polygon.
fn facing(vertices: &[Vec2], m: Vec2, across: Vec2) -> (f32, f32, f32) {
    /// As a fraction of the polygon's size.
    const TOLERANCE: f32 = 1e-5;
    let level = vertices
        .iter()
        .map(|p| p.dot(m))
        .fold(f32::NEG_INFINITY, f32::max);
    let size = vertices
        .iter()
        .map(|p| p.abs().max_element())
        .fold(0.0, f32::max);
    let (lo, hi) = vertices
        .iter()
        .filter(|p| p.dot(m) >= level - TOLERANCE * size)
        .fold((f32::INFINITY, f32::NEG_INFINITY), |(lo, hi), p| {
            let d = p.dot(across);
            (lo.min(d), hi.max(d))
        });
    (lo, hi, level)
}

/// The vertex of `vertices` furthest along `dir`.
fn furthest(vertices: &[Vec2], dir: Vec2) -> Vec2 {
    let furthest = vertices
        .iter()
        .max_by(|p, q| p.dot(dir).total_cmp(&q.dot(dir)));
    *furthest.expect("a polygon has vertices")
}

/// The vertex of `vertices` nearest to `p`.
fn nearest_vertex(vertices: &[Vec2], p: Vec2) -> Vec2 {
    let nearest = vertices
        .iter()
        .min_by(|q, r| q.distance_squared(p).total_cmp(&r.distance_squared(p)));
    *nearest.expect("a polygon has vertices")
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

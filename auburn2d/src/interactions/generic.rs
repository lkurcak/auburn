//! Whether two colliders overlap, and the shortest move that parts them: [`Collides`] and
//! [`Penetrates`] between any two colliders, whatever their shapes and transformations.
//!
//! Built-in shapes that their transformations keep as they are (see [`Shape::primitive`] and
//! [`Transformation2d::rotation_and_scale`]) have exact formulas: points, balls, boxes and
//! polygons against each other. Every other pair goes through GJK, and EPA for the way out,
//! on the colliders' support functions.
//!
//! Touching counts as overlapping, as in the casts: shapes that only touch collide, and the
//! shortest move that parts them is zero. Where the pair has a formula, `penetrates` gives a
//! vector exactly when `collides` is true, and casts at rest find the shapes overlapping exactly
//! then too, but for a ball against a polygon, which casts through support functions.
//!
//! The formulas are `#[inline]`, as in the casts, and the match on the pair's forms, with the
//! ways out that it picks for the basic pairs, is `#[inline(always)]`: LLVM keeps it as a call
//! otherwise, and `collides` takes four times as long.

use crate::Vec2;
use crate::algorithm::{epa, gjk};
use crate::collider::Collider;
use crate::interactions::cast::{Form, Rect, each_axis, overlap};
use crate::property::support::Support;
use crate::relation::cast::{against, unit};
use crate::relation::collides::Collides;
use crate::relation::penetrates::Penetrates;
use crate::shape::Shape;
use crate::transformation::Transformation2d;

const GJK_MAX_ITERATIONS: usize = 30;

impl<S1, S2, T1, T2> Collides<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    #[inline]
    fn collides(&self, other: &Collider<'_, S2, T2>) -> bool {
        match overlaps(&Form::of(self), &Form::of(other)) {
            Some(overlaps) => overlaps,
            None => gjk(
                |dir| self.support(dir),
                |dir| other.support(dir),
                GJK_MAX_ITERATIONS,
            )
            .is_some(),
        }
    }
}

impl<S1, S2, T1, T2> Penetrates<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    fn penetrates(&self, other: &Collider<'_, S2, T2>) -> Option<Vec2> {
        let (x, y) = (Form::of(self), Form::of(other));
        let Some(overlaps) = overlaps(&x, &y) else {
            let (support_a, support_b) = (|dir| self.support(dir), |dir| other.support(dir));
            let simplex = gjk(support_a, support_b, GJK_MAX_ITERATIONS)?;
            // EPA's vector pushes `other` away from `self`.
            return Some(-epa(simplex, support_a, support_b));
        };
        overlaps.then(|| {
            let (normal, depth) = penetration(&x, &y, Vec2::ZERO);
            normal * depth
        })
    }
}

/// Whether `a` overlaps `b` (touching counts), by the pair's exact formula: `None` if it has
/// none.
#[inline(always)]
fn overlaps(a: &Form<'_>, b: &Form<'_>) -> Option<bool> {
    use Form::{Ball, Other, Point, Rect};
    Some(match (a, b) {
        (Other, _) | (_, Other) => return None,
        (Point(p), Point(q)) => p == q,
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => in_ball(*p - *q, *r),
        (Ball(p, r), Ball(q, s)) => in_ball(*p - *q, r + s),
        (Ball(p, r), Rect(rect)) | (Rect(rect), Ball(p, r)) => ball_in_rect(*p, *r, rect),
        (Point(p), Rect(rect)) | (Rect(rect), Point(p)) => in_aabb(rect.local(*p), rect.half),
        // Turned the same way, `a` overlaps `b` while its centre is in a box with both half
        // sizes.
        (Rect(r), Rect(s)) if r.rot == s.rot => in_aabb(s.local(r.centre), r.half + s.half),
        _ => separating_axes_overlap(a, b),
    })
}

/// Whether a point at `p`, relative to the centre of a ball of `radius`, is in it.
#[inline]
pub(super) fn in_ball(p: Vec2, radius: f32) -> bool {
    p.length_squared() <= radius * radius
}

/// Whether a point at `p`, relative to the centre of a box of half size `half` that doesn't
/// turn, is in it.
#[inline]
fn in_aabb(p: Vec2, half: Vec2) -> bool {
    p.abs().cmple(half).all()
}

/// Whether a ball at `centre` overlaps `rect`: whether its centre is within `radius` of the
/// box.
#[inline]
pub(super) fn ball_in_rect(centre: Vec2, radius: f32, rect: &Rect) -> bool {
    in_rounded_aabb(rect.local(centre), rect.half, radius)
}

/// Whether a point at `p`, relative to the center of a box of half size `half` that doesn't
/// turn, is within `radius` of it.
#[inline]
pub(super) fn in_rounded_aabb(p: Vec2, half: Vec2, radius: f32) -> bool {
    let outside = (p.abs() - half).max(Vec2::ZERO);
    outside.length_squared() <= radius * radius
}

/// Whether `a` overlaps `b` by the separating axis theorem: whether their spans overlap
/// along every axis that could separate them. Each is a point, ball, box or polygon, and
/// [`each_axis`] says which axes those are.
fn separating_axes_overlap(a: &Form<'_>, b: &Form<'_>) -> bool {
    // Measured from `b`'s anchor, so that the spans stay as precise as the shapes far from
    // the origin.
    let offset = a.anchor() - b.anchor();
    each_axis(a, b, |axis| {
        let (lo, hi) = overlap(a, b, offset, axis);
        (lo <= 0.0 && 0.0 <= hi).then_some(())
    })
    .is_some()
}

/// How to part `a` from `b`, which it overlaps now, by the shortest move: the unit normal out
/// of `b`, and how far. Where every direction parts them as soon, as for a point at a ball's
/// centre or on another point, the normal is against `vel`, or +x at rest. Only for pairs with
/// an exact formula.
#[inline(always)]
pub(super) fn penetration(a: &Form<'_>, b: &Form<'_>, vel: Vec2) -> (Vec2, f32) {
    use Form::{Ball, Point, Rect};
    match (a, b) {
        (Point(_), Point(_)) => (against(vel), 0.0),
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => point_ball_penetration(*p - *q, *r, vel),
        (Ball(p, r), Ball(q, s)) => point_ball_penetration(*p - *q, r + s, vel),
        (Ball(p, r), Rect(rect)) => ball_rect_penetration(*p, *r, rect),
        (Rect(rect), Ball(p, r)) => {
            let (normal, depth) = ball_rect_penetration(*p, *r, rect);
            (-normal, depth)
        }
        (Point(p), Rect(rect)) => aligned_penetration(*p, rect.half, rect),
        (Rect(rect), Point(p)) => {
            let (normal, depth) = aligned_penetration(*p, rect.half, rect);
            (-normal, depth)
        }
        (Rect(r), Rect(s)) => rect_rect_penetration(r, s),
        _ => separating_axes_penetration(a, b),
    }
}

/// How to part a point at `p`, relative to the centre of a ball of `radius` that holds it, by
/// the shortest move: straight out from the centre, or against `vel` from the centre itself.
#[inline(always)]
fn point_ball_penetration(p: Vec2, radius: f32, vel: Vec2) -> (Vec2, f32) {
    match length_and_direction(p) {
        Some((length, normal)) => (normal, radius - length),
        None => (against(vel), radius),
    }
}

/// How to part box `a` from box `b`, which it overlaps, by the shortest move.
#[inline(always)]
fn rect_rect_penetration(a: &Rect, b: &Rect) -> (Vec2, f32) {
    if a.rot != b.rot {
        return separating_axes_penetration(&Form::Rect(*a), &Form::Rect(*b));
    }
    // Out of the box with both half sizes.
    aligned_penetration(a.centre, a.half + b.half, b)
}

/// How to part a point at `p` from a box turned as `rect` is, at its center, with half size
/// `half`, which holds it, by the shortest move: through its nearest side.
#[inline(always)]
fn aligned_penetration(p: Vec2, half: Vec2, rect: &Rect) -> (Vec2, f32) {
    let (normal, depth) = out_of_aabb(rect.local(p), half);
    (rect.in_world(normal), depth)
}

/// How to part a ball at `centre` from `rect`, which it overlaps, by the shortest move: out
/// from the nearest point of the box, or if the centre is in the box, through its nearest side.
#[inline(always)]
fn ball_rect_penetration(centre: Vec2, radius: f32, rect: &Rect) -> (Vec2, f32) {
    let p = rect.local(centre);
    let half = rect.half;
    let outside = p - p.max(-half).min(half);
    let (normal, depth) = match length_and_direction(outside) {
        Some((length, normal)) => (normal, radius - length),
        None => {
            let (normal, depth) = out_of_aabb(p, half);
            (normal, radius + depth)
        }
    };
    (rect.in_world(normal), depth)
}

/// The way out of a box of half size `half` that doesn't turn for a point at `p` in it,
/// relative to its centre: through its nearest side, and how far.
#[inline(always)]
fn out_of_aabb(p: Vec2, half: Vec2) -> (Vec2, f32) {
    nearest([
        (Vec2::X, half.x - p.x),
        (Vec2::NEG_X, half.x + p.x),
        (Vec2::Y, half.y - p.y),
        (Vec2::NEG_Y, half.y + p.y),
    ])
}

/// How to part `a` from `b`, which it overlaps, by the shortest move, by the separating axis
/// theorem: along the axis on which their spans overlap the least. Each is a point, ball, box
/// or polygon, and [`each_axis`] says which axes could separate them.
fn separating_axes_penetration(a: &Form<'_>, b: &Form<'_>) -> (Vec2, f32) {
    let offset = a.anchor() - b.anchor();
    let mut best = (Vec2::X, f32::INFINITY);
    each_axis(a, b, |axis| {
        let Some(axis) = unit(axis) else {
            return Some(());
        };
        let (lo, hi) = overlap(a, b, offset, axis);
        best = nearest([best, (axis, hi), (-axis, -lo)]);
        Some(())
    });
    best
}

/// The way out of `ways`, each a direction and how far to go that way, that goes the least far:
/// the first of those that go as little.
#[inline(always)]
fn nearest<const N: usize>(ways: [(Vec2, f32); N]) -> (Vec2, f32) {
    // A loop, which LLVM inlines whole, where `reduce` leaves a call.
    let mut best = ways[0];
    for way in ways {
        if way.1 < best.1 {
            best = way;
        }
    }
    best
}

/// The length of `v` and its direction, a unit vector, or `None` if it is too short to have a
/// direction.
#[inline(always)]
pub(super) fn length_and_direction(v: Vec2) -> Option<(f32, Vec2)> {
    let squared = v.length_squared();
    // A square that neither underflows nor overflows has an accurate root.
    if squared.is_normal() {
        let length = squared.sqrt();
        return Some((length, v / length));
    }
    let direction = unit(v)?;
    Some((v.dot(direction), direction))
}

//! Whether two colliders overlap, and the shortest move that parts them: [`Collides`] and
//! [`Penetrates`] between any two colliders, whatever their shapes and transformations.
//!
//! Built-in shapes that their transformations keep as they are (see [`Shape::primitive`] and
//! [`Transformation3d::rotation_and_scale`]) have exact formulas: points, balls and boxes
//! against each other, except two boxes turned different ways. Every other pair goes through
//! GJK, and EPA for the way out, on the colliders' support functions.
//!
//! Touching counts as overlapping, as in the casts: shapes that only touch collide, and the
//! shortest move that parts them is zero. Where the pair has a formula, `penetrates` gives a
//! vector exactly when `collides` is true.
//!
//! The formulas are `#[inline]`, as in the casts, and the match on the pair's forms is
//! `#[inline(always)]`: LLVM keeps it as a call otherwise, and `collides` takes four times as
//! long.

use crate::Vec3;
use crate::algorithm::{epa, gjk};
use crate::collider::Collider;
use crate::interactions::cast::{Cuboid, Form};
use crate::property::support::Support;
use crate::relation::cast::unit;
use crate::relation::collides::Collides;
use crate::relation::penetrates::Penetrates;
use crate::shape::Shape;
use crate::transformation::Transformation3d;

const GJK_MAX_ITERATIONS: usize = 30;

impl<S1, S2, T1, T2> Collides<Collider<'_, S2, T2>> for Collider<'_, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation3d,
    T2: Transformation3d,
{
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
    T1: Transformation3d,
    T2: Transformation3d,
{
    fn penetrates(&self, other: &Collider<'_, S2, T2>) -> Option<Vec3> {
        let (x, y) = (Form::of(self), Form::of(other));
        let Some(overlaps) = overlaps(&x, &y) else {
            let (support_a, support_b) = (|dir| self.support(dir), |dir| other.support(dir));
            let simplex = gjk(support_a, support_b, GJK_MAX_ITERATIONS)?;
            // EPA's vector pushes `other` away from `self`.
            return Some(-epa(simplex, support_a, support_b));
        };
        overlaps.then(|| penetration(&x, &y))
    }
}

/// Whether `a` overlaps `b` (touching counts), by the pair's exact formula: `None` if it has
/// none.
#[inline(always)]
fn overlaps(a: &Form, b: &Form) -> Option<bool> {
    use Form::{Ball, Cuboid, Point};
    Some(match (a, b) {
        (Point(p), Point(q)) => p == q,
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => in_ball(*p - *q, *r),
        (Ball(p, r), Ball(q, s)) => in_ball(*p - *q, r + s),
        (Point(p), Cuboid(cuboid)) | (Cuboid(cuboid), Point(p)) => in_cuboid(*p, cuboid),
        (Ball(p, r), Cuboid(cuboid)) | (Cuboid(cuboid), Ball(p, r)) => {
            ball_in_cuboid(*p, *r, cuboid)
        }
        // Turned the same way, `a` overlaps `b` while its centre is in a box with both half
        // sizes.
        (Cuboid(a), Cuboid(b)) if a.rot == b.rot => in_aabb(b.local(a.centre), a.half + b.half),
        _ => return None,
    })
}

/// Whether a point at `p`, relative to the centre of a ball of `radius`, is in it.
#[inline]
pub(super) fn in_ball(p: Vec3, radius: f32) -> bool {
    p.length_squared() <= radius * radius
}

/// Whether a point at `p` is in `cuboid`.
#[inline]
pub(super) fn in_cuboid(p: Vec3, cuboid: &Cuboid) -> bool {
    in_aabb(cuboid.local(p), cuboid.half)
}

/// Whether a point at `p`, relative to the centre of a box of half size `half` that doesn't
/// turn, is in it.
#[inline]
fn in_aabb(p: Vec3, half: Vec3) -> bool {
    p.abs().cmple(half).all()
}

/// Whether a ball at `centre` overlaps `cuboid`: whether its centre is within `radius` of the
/// box.
#[inline]
fn ball_in_cuboid(centre: Vec3, radius: f32, cuboid: &Cuboid) -> bool {
    let outside = (cuboid.local(centre).abs() - cuboid.half).max(Vec3::ZERO);
    outside.length_squared() <= radius * radius
}

/// The shortest move that parts `a` from `b`, which it overlaps now. Where every direction
/// parts them as soon, as for a point at a ball's centre, it is along +x. Only for pairs with
/// an exact formula.
#[inline(always)]
fn penetration(a: &Form, b: &Form) -> Vec3 {
    use Form::{Ball, Cuboid, Point};
    match (a, b) {
        (Point(_), Point(_)) => Vec3::ZERO,
        (Point(p), Ball(q, r)) | (Ball(p, r), Point(q)) => out_of_ball(*p - *q, *r),
        (Ball(p, r), Ball(q, s)) => out_of_ball(*p - *q, r + s),
        (Point(p), Cuboid(cuboid)) => point_cuboid_penetration(*p, cuboid),
        (Cuboid(cuboid), Point(p)) => -point_cuboid_penetration(*p, cuboid),
        (Ball(p, r), Cuboid(cuboid)) => ball_cuboid_penetration(*p, *r, cuboid),
        (Cuboid(cuboid), Ball(p, r)) => -ball_cuboid_penetration(*p, *r, cuboid),
        (Cuboid(a), Cuboid(b)) => {
            let (normal, depth) = out_of_aabb(b.local(a.centre), a.half + b.half);
            b.rot * (normal * depth)
        }
        _ => unreachable!("the pair has no formula"),
    }
}

/// The shortest move that parts a point at `p`, relative to the centre of a ball of `radius`
/// that holds it, from the ball: straight out from the centre, or along +x from the centre
/// itself.
#[inline]
fn out_of_ball(p: Vec3, radius: f32) -> Vec3 {
    match length_and_direction(p) {
        Some((length, direction)) => direction * (radius - length),
        None => Vec3::new(radius, 0.0, 0.0),
    }
}

/// The shortest move that parts a point at `p` from `cuboid`, which holds it: through its
/// nearest face.
#[inline]
fn point_cuboid_penetration(p: Vec3, cuboid: &Cuboid) -> Vec3 {
    let (normal, depth) = out_of_aabb(cuboid.local(p), cuboid.half);
    cuboid.rot * (normal * depth)
}

/// The shortest move that parts a ball at `centre` from `cuboid`, which it overlaps: out from
/// the nearest point of the box, or if the centre is in the box, through its nearest face.
#[inline]
fn ball_cuboid_penetration(centre: Vec3, radius: f32, cuboid: &Cuboid) -> Vec3 {
    let p = cuboid.local(centre);
    let half = cuboid.half;
    let outside = p - p.max(-half).min(half);
    let local = match length_and_direction(outside) {
        Some((length, direction)) => direction * (radius - length),
        None => {
            let (normal, depth) = out_of_aabb(p, half);
            normal * (radius + depth)
        }
    };
    cuboid.rot * local
}

/// The way out of a box of half size `half` that doesn't turn for a point at `p` in it,
/// relative to its centre: through its nearest face, and how far.
#[inline]
fn out_of_aabb(p: Vec3, half: Vec3) -> (Vec3, f32) {
    nearest([
        (Vec3::X, half.x - p.x),
        (Vec3::NEG_X, half.x + p.x),
        (Vec3::Y, half.y - p.y),
        (Vec3::NEG_Y, half.y + p.y),
        (Vec3::Z, half.z - p.z),
        (Vec3::NEG_Z, half.z + p.z),
    ])
}

/// The way out of `ways`, each a direction and how far to go that way, that goes the least far:
/// the first of those that go as little.
#[inline]
fn nearest<const N: usize>(ways: [(Vec3, f32); N]) -> (Vec3, f32) {
    ways.into_iter()
        .reduce(|best, way| if way.1 < best.1 { way } else { best })
        .expect("there is a way")
}

/// The length of `v` and its direction, a unit vector, or `None` if it is too short to have a
/// direction.
#[inline]
fn length_and_direction(v: Vec3) -> Option<(f32, Vec3)> {
    let squared = v.length_squared();
    // A square that neither underflows nor overflows has an accurate root.
    if squared.is_normal() {
        let length = squared.sqrt();
        return Some((length, v / length));
    }
    let direction = unit(v)?;
    Some((v.dot(direction), direction))
}

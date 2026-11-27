//! Casts between any two convex shapes, given their support functions.
//!
//! A shape `a` moving at `vel` overlaps a shape `b` at time `t` when `t * vel` is in the
//! Minkowski difference `b - a`: the offsets by which `a` would have to move to touch `b`.
//! That set is convex, so the times form one interval, where the line through the origin along
//! `vel` crosses it. [`shape_cast`] finds both ends of it with a GJK ray cast (G. van den
//! Bergen, "Ray Casting against General Convex Objects with Application to Continuous
//! Collision Detection", 2004), one from each side.
//!
//! The pairs with a ball cast it as its centre and its radius. The set is then a polytope
//! rounded by that radius: the cast resolves the polytope exactly and steps onto the rounding
//! by Newton's method, and its distances are never shorter than the radius, so rounding errors
//! in the support points don't swamp them.
//!
//! Its tolerances are relative to the shapes, so it works the same for tiny and huge ones.

use super::gjk::gjk;
use crate::Vec3;
use crate::relation::cast::{direction_and_speed, is_at_rest};

const GJK_MAX_ITERATIONS: usize = 30;
const RAY_CAST_MAX_ITERATIONS: usize = 64;
/// A ray cast hits once it is no further from the entry than this fraction of the set's size,
/// as far as it can tell from the distance to the set and the angle the ray meets it at.
const RELATIVE_TOLERANCE: f32 = 1e-5;
/// Far from the origin, rounding in the support points' coordinates outweighs that, so a ray
/// cast also hits within this many units of f32 precision of those coordinates.
const ROUNDING_TOLERANCE: f32 = 16.0 * f32::EPSILON;
/// A ray cast that rounding keeps from getting any closer hits if it is this close, as a
/// fraction of the set's size.
const STALL_TOLERANCE: f32 = 1e-3;

/// When shape `a`, moving at `vel`, overlaps shape `b`, over all of time (touching counts):
/// from when to when, or `None` if never.
///
/// `support_a` and `support_b` are the shapes' support functions at time 0, in the same frame
/// as `vel`. At rest, shapes that overlap do so for all of time, from −∞ to ∞.
///
/// # Examples
///
/// ```
/// use auburn3d::Vec3;
/// use auburn3d::algorithm::shape_cast;
///
/// // A unit ball at the origin, moving along x at 2, and a unit ball at (10, 0, 0).
/// let a = |dir: Vec3| dir.normalize_or_zero();
/// let b = |dir: Vec3| Vec3::new(10.0, 0.0, 0.0) + dir.normalize_or_zero();
/// let (entry, exit) = shape_cast(a, b, Vec3::new(2.0, 0.0, 0.0)).unwrap();
/// assert!((entry - 4.0).abs() < 1e-4 && (exit - 6.0).abs() < 1e-4);
/// ```
pub fn shape_cast<F1, F2>(support_a: F1, support_b: F2, vel: Vec3) -> Option<(f32, f32)>
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    rounded_shape_cast(support_a, support_b, 0.0, vel)
}

/// [`shape_cast`] for shapes `a` and `b` rounded by balls whose radii sum to `radius`: what
/// `support_a` and `support_b` describe are their cores. A ball is a point rounded by its
/// radius.
pub(crate) fn rounded_shape_cast<F1, F2>(
    support_a: F1,
    support_b: F2,
    radius: f32,
    vel: Vec3,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    if is_at_rest(vel) {
        let rounded_b = |dir: Vec3| support_b(dir) + radius * dir.normalize_or_zero();
        let overlap = gjk(&support_a, rounded_b, GJK_MAX_ITERATIONS);
        return overlap.map(|_| (f32::NEG_INFINITY, f32::INFINITY));
    }
    let support = |dir: Vec3| support_b(dir) - support_a(-dir);
    let (dir, speed) = direction_and_speed(vel);
    // Along `dir`, the set spans [lo, hi], so the line can only be in it there.
    let lo = support(-dir).dot(dir) - radius;
    let hi = support(dir).dot(dir) + radius;
    let cast = |dir, from, to| first_along(support, radius, dir, from, to, hi - lo);
    let entry = cast(dir, lo, hi)?;
    // Rounding can make the other side miss a graze, which enters and exits at once.
    let exit = cast(-dir, -hi, -lo).map_or(entry, |t| -t);
    Some((entry / speed, exit.max(entry) / speed))
}

/// The least `t` in [`from`, `to`] at which `t * dir` is within `radius` of the convex set
/// that `support` describes, or `None` if there is none. `dir` is a unit vector, and the
/// rounded set lies beyond `from` along it: `from` is at most the least `p.dot(dir)` of a
/// point `p` in it. `extent` is the rounded set's length along `dir`.
fn first_along(
    support: impl Fn(Vec3) -> Vec3,
    radius: f32,
    dir: Vec3,
    from: f32,
    to: f32,
    extent: f32,
) -> Option<f32> {
    let mut t = from;
    let mut x = t * dir;
    let mut simplex = Simplex::new(support(-dir));
    // From `x` to the point of the simplex nearest to it.
    let mut near = simplex.nearest(x);
    for _ in 0..RAY_CAST_MAX_ITERATIONS {
        let distance = near.length();
        // `x` is at most this far outside the rounded set, along `near`.
        let excess = distance - radius;
        let scale = simplex.size(x).max(extent);
        let rounding = simplex.rounding(x, radius);
        if excess <= rounding {
            return Some(t);
        }
        let normal = near / distance;
        // Met at an angle, a ray still has `excess / approach` to go to reach the set.
        let approach = dir.dot(normal);
        if excess <= RELATIVE_TOLERANCE * scale * approach {
            return Some(t);
        }
        let p = support(-near);
        // All of the rounded set is at least this far along `normal` from `x`.
        let gap = (p - x).dot(normal) - radius;
        let before = (t, distance);
        if gap > 0.0 {
            // The plane `gap` along `normal` separates `x` from the set, so the ray either
            // misses or crosses it before reaching the set.
            if approach <= 0.0 {
                return None;
            }
            t += gap / approach;
            if t > to {
                return None;
            }
            x = t * dir;
        }
        simplex.push(p);
        near = simplex.nearest(x);
        // Without rounding, each step either moves `x` or brings the simplex closer to it.
        if t == before.0 && near.length() >= before.1 {
            break;
        }
    }
    let excess = near.length() - radius;
    let scale = simplex.size(x).max(extent);
    (excess <= STALL_TOLERANCE * scale).then_some(t)
}

/// Up to four points of a convex set, the fewest whose hull holds the point of it nearest to
/// the ray's current point.
struct Simplex {
    points: [Vec3; 4],
    len: usize,
}

/// The faces of a tetrahedron, each with the corner opposite it last.
const FACES: [[usize; 4]; 4] = [[0, 1, 2, 3], [0, 1, 3, 2], [0, 2, 3, 1], [1, 2, 3, 0]];

impl Simplex {
    fn new(p: Vec3) -> Self {
        Self {
            points: [p; 4],
            len: 1,
        }
    }

    /// The simplex of `self`'s points at `indices`.
    fn of(&self, indices: &[usize]) -> Self {
        let mut part = Self::new(self.points[indices[0]]);
        for &i in &indices[1..] {
            part.push(self.points[i]);
        }
        part
    }

    fn push(&mut self, p: Vec3) {
        // `nearest` keeps at most three points unless the hull holds `x`, which ends the cast.
        debug_assert!(self.len < 4);
        self.points[self.len] = p;
        self.len += 1;
    }

    /// How far the simplex's points are from `x`, at most.
    fn size(&self, x: Vec3) -> f32 {
        let points = &self.points[..self.len];
        points.iter().map(|&p| p.distance(x)).fold(0.0, f32::max)
    }

    /// How far from `x` rounding alone can put the simplex's nearest point: a few units of f32
    /// precision in its coordinates, in `x`'s and in `radius`.
    fn rounding(&self, x: Vec3, radius: f32) -> f32 {
        let points = &self.points[..self.len];
        let magnitude = points.iter().map(|p| p.length());
        let magnitude = magnitude.fold(x.length().max(radius), f32::max);
        ROUNDING_TOLERANCE * magnitude
    }

    /// The vector from `x` to the point of the simplex's hull nearest to it. Drops the points
    /// not needed to hold that point, unless the hull holds `x`, where it returns zero.
    fn nearest(&mut self, x: Vec3) -> Vec3 {
        let q = self.points.map(|p| p - x);
        let (near, part) = match self.len {
            1 => return q[0],
            2 => self.nearest_on_edge(q, 0, 1),
            3 => self.nearest_on_triangle(q, [0, 1, 2]),
            _ => {
                let tolerance = RELATIVE_TOLERANCE * self.size(x);
                if contains_origin(q, tolerance.max(self.rounding(x, 0.0))) {
                    return Vec3::ZERO;
                }
                FACES
                    .map(|[i, j, k, _]| self.nearest_on_triangle(q, [i, j, k]))
                    .into_iter()
                    .min_by(|a, b| a.0.length_squared().total_cmp(&b.0.length_squared()))
                    .expect("a tetrahedron has faces")
            }
        };
        *self = part;
        near
    }

    /// The point nearest to the origin on the triangle of `q`'s points at `corners`, the
    /// simplex's points seen from `x`, and the fewest of those points that hold it.
    fn nearest_on_triangle(&self, q: [Vec3; 4], corners: [usize; 3]) -> (Vec3, Self) {
        let [i, j, k] = corners;
        let normal = (q[j] - q[i]).cross(q[k] - q[i]);
        let normal_squared = normal.length_squared();
        // Seen along the normal, the origin is inside if it is on the inner side of every edge.
        let inside = normal_squared > 0.0
            && [(i, j), (j, k), (k, i)]
                .iter()
                .all(|&(a, b)| (q[b] - q[a]).cross(-q[a]).dot(normal) >= 0.0);
        if inside {
            // Across the face, which keeps it accurate when the face is large.
            let near = normal * (normal.dot(q[i]) / normal_squared);
            return (near, self.of(&corners));
        }
        [(i, j), (j, k), (k, i)]
            .map(|(a, b)| self.nearest_on_edge(q, a, b))
            .into_iter()
            .min_by(|a, b| a.0.length_squared().total_cmp(&b.0.length_squared()))
            .expect("a triangle has edges")
    }

    /// The point nearest to the origin on the edge from `q[i]` to `q[j]`, the simplex's points
    /// seen from `x`, and that edge's points, or the one corner if the point is there.
    fn nearest_on_edge(&self, q: [Vec3; 4], i: usize, j: usize) -> (Vec3, Self) {
        let (a, b) = (q[i], q[j]);
        let edge = b - a;
        if edge.dot(-a) <= 0.0 {
            (a, self.of(&[i]))
        } else if edge.dot(-b) >= 0.0 {
            (b, self.of(&[j]))
        } else {
            // Across the edge, which keeps it accurate when the edge is long.
            let near = edge.cross(a.cross(edge)) / edge.length_squared();
            (near, self.of(&[i, j]))
        }
    }
}

/// Whether the tetrahedron `q` holds the origin, touching counts: the origin is on the same
/// side of each face as the corner opposite it, or on the face.
///
/// Each corner has to stand more than `tolerance` off the face opposite it. A flatter
/// tetrahedron is one that rounding can turn inside out, and a point inside it is within
/// `tolerance` of a face anyway, so its faces decide.
fn contains_origin(q: [Vec3; 4], tolerance: f32) -> bool {
    FACES.iter().all(|&[i, j, k, l]| {
        let normal = (q[j] - q[i]).cross(q[k] - q[i]);
        let corner = normal.dot(q[l] - q[i]);
        let origin = normal.dot(-q[i]);
        corner.abs() > tolerance * normal.length()
            && (origin == 0.0 || (origin > 0.0) == (corner > 0.0))
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::Quat;
    use crate::prelude::*;

    fn near_span(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
        got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-3 && (b - want.1).abs() < 1e-3)
    }

    fn ball(centre: Vec3, radius: f32) -> impl Fn(Vec3) -> Vec3 {
        move |dir: Vec3| centre + dir.normalize_or_zero() * radius
    }

    fn cube(centre: Vec3, half: f32) -> impl Fn(Vec3) -> Vec3 {
        move |dir: Vec3| {
            centre
                + Vec3::new(
                    half.copysign(dir.x),
                    half.copysign(dir.y),
                    half.copysign(dir.z),
                )
        }
    }

    fn near(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
        got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-4 && (b - want.1).abs() < 1e-4)
    }

    #[test]
    fn balls_meet_and_part() {
        let a = ball(Vec3::ZERO, 1.0);
        let b = ball(Vec3::new(3.0, 0.0, 0.0), 1.0);
        let got = shape_cast(a, b, Vec3::X);
        assert!(near(got, (1.0, 5.0)), "{got:?}");
    }

    #[test]
    fn balls_miss() {
        let a = ball(Vec3::ZERO, 1.0);
        let b = ball(Vec3::new(3.0, 0.0, 3.0), 1.0);
        assert_eq!(shape_cast(a, b, Vec3::X), None);
    }

    #[test]
    fn overlap_now_started_in_the_past() {
        let a = ball(Vec3::ZERO, 1.0);
        let b = ball(Vec3::new(0.5, 0.0, 0.0), 1.0);
        let got = shape_cast(a, b, Vec3::X);
        assert!(near(got, (-1.5, 2.5)), "{got:?}");
    }

    #[test]
    fn far_beyond_time_one() {
        let a = cube(Vec3::ZERO, 1.0);
        let b = cube(Vec3::new(10.0, 0.0, 0.0), 1.0);
        let got = shape_cast(a, b, Vec3::X);
        assert!(near(got, (8.0, 12.0)), "{got:?}");
    }

    #[test]
    fn ball_meets_corner() {
        // The ball's centre runs along the x axis and meets the cube's corner (2, 0.5, 0.5)
        // after travelling 2 - √0.5, and leaves its corner (4, 0.5, 0.5) after 4 + √0.5.
        let a = ball(Vec3::ZERO, 1.0);
        let b = cube(Vec3::new(3.0, 1.5, 1.5), 1.0);
        let got = shape_cast(a, b, Vec3::new(2.0, 0.0, 0.0));
        let s = 0.5f32.sqrt();
        assert!(near(got, ((2.0 - s) / 2.0, (4.0 + s) / 2.0)), "{got:?}");
    }

    #[test]
    fn ball_meets_edge() {
        // The cube's edge from (2, 0.5, -1) to (2, 0.5, 1) is half a unit off the path.
        let a = ball(Vec3::ZERO, 1.0);
        let b = cube(Vec3::new(3.0, 1.5, 0.0), 1.0);
        let got = shape_cast(a, b, Vec3::X);
        let s = 0.75f32.sqrt();
        assert!(near(got, (2.0 - s, 4.0 + s)), "{got:?}");
    }

    #[test]
    fn same_at_any_scale() {
        for k in [1e-4, 1.0, 1e4] {
            let a = ball(Vec3::ZERO, k);
            let b = cube(Vec3::new(3.0, 1.5, 1.5) * k, k);
            let (entry, exit) = shape_cast(a, b, Vec3::new(2.0, 0.0, 0.0) * k).unwrap();
            let s = 0.5f32.sqrt();
            let want = ((2.0 - s) / 2.0, (4.0 + s) / 2.0);
            assert!(
                (entry - want.0).abs() < 1e-4 && (exit - want.1).abs() < 1e-4,
                "{k}: ({entry}, {exit}), want {want:?}"
            );
        }
    }

    #[test]
    fn at_rest() {
        let a = ball(Vec3::ZERO, 1.0);
        let b = ball(Vec3::new(1.5, 0.0, 0.0), 1.0);
        let c = ball(Vec3::new(3.0, 0.0, 0.0), 1.0);
        let all_time = Some((f32::NEG_INFINITY, f32::INFINITY));
        assert_eq!(shape_cast(&a, b, Vec3::ZERO), all_time);
        assert_eq!(shape_cast(&a, c, Vec3::ZERO), None);
    }

    // The cases below came up in the randomized tests (`tests/randomized`) at 200 times the
    // cases, each with a ray cast that got it wrong.

    /// Once the ray reaches a face of the boxes' Minkowski difference, the simplex lies flat on
    /// that face, and rounding can turn the flat tetrahedron inside out around a point outside.
    #[test]
    fn box_box_cast_on_a_face() {
        let ta = Transform3d {
            pos: Vec3::new(-0.759_668_1, -2.809_233_2, 0.105_212_21),
            rot: Quat::from_xyzw(0.645_306_7, -0.057_835_672, -0.252_602_28, 0.718_628),
            scale: 1.104_294_5,
        };
        let tb = Transform3d {
            pos: Vec3::new(-1.808_111_8, -1.845_061_5, -2.029_615_4),
            rot: Quat::from_xyzw(-0.844_821_93, 0.016_985_75, -0.479_130_48, -0.237_531_96),
            scale: 2.795_428_3,
        };
        let a = Box3d::new(Vec3::new(1.731_739_8, 0.532_804_37, 1.587_559));
        let b = Box3d::new(Vec3::new(0.231_591_61, 0.520_357_55, 0.760_913_5));
        let r = a
            .at(&ta)
            .tttoiae(&b.at(&tb), Vec3::new(-9.778_374, 0.0, 0.0));
        assert!(near_span(r, (0.111_37, 0.572_12)), "{r:?}");
    }

    /// A ball's support points, a box corner plus a radius, are far from where the ray meets a
    /// rounded edge, and rounding in them swamps the steps the ray cast has left to take: it
    /// stalls short of the shapes and misses, unless the ball is its centre and a radius.
    #[test]
    fn ball_box_cast_at_a_rounded_edge() {
        let ta = Transform3d {
            pos: Vec3::new(-1.845_870_5, 1.825_297_4, 0.437_979_7),
            rot: Quat::from_xyzw(-0.324_404_98, 0.793_659_15, -0.060_177_423, -0.511_121_4),
            scale: 1.0,
        };
        let tb = Transform3d {
            pos: Vec3::new(-1.551_081_4, 1.880_300_5, 0.315_365_8),
            rot: Quat::from_xyzw(0.651_709_4, -0.395_632_9, 0.567_148_27, -0.311_596_16),
            scale: 1.0,
        };
        let ball = Ball::new(0.415_914_83);
        let cube = Box3d::new(Vec3::new(1.914_785_3, 1.989_441_6, 1.134_578_6));
        let vel = Vec3::new(-0.195_181_73, 0.286_369_83, -0.076_545);
        let r = ball.at(&ta).toiae(&cube.at(&tb), vel);
        assert!(near_span(r, (0.0, 6.464_48)), "{r:?}");
        let r = Box3d::new(Vec3::new(0.721_902_67, 0.442_961_36, 1.599_202_3))
            .at(&Vec3::new(2.931_282_5, 0.518_697, -0.862_559_1))
            .toiae(
                &Ball::new(0.894_420_7).at(&Vec3::new(1.098_608, 1.099_615_1, 2.884_670_3)),
                Vec3::new(-2.001_304_6, 0.453_286_05, 5.826_924_3),
            );
        assert!(near_span(r, (0.254_95, 1.071_04)), "{r:?}");
    }

    /// A ray that meets a face at a shallow angle passes close to the box well before it enters
    /// it: being close isn't having arrived, which here is 0.02 further on.
    #[test]
    fn point_box_cast_at_a_shallow_angle() {
        let cube = Box3d::new(Vec3::new(1.326_207_3, 1.880_736_4, 0.657_051_6));
        let r = Point
            .at(&Vec3::new(-1.725_609_2, -2.397_362_2, -1.670_809_1))
            .tttoiae(
                &cube.at(&Vec3::new(2.451_517, 0.313_199_04, -1.009_139_3)),
                Vec3::new(6.647_353, 4.299_819, 0.010_704_815),
            );
        assert!(
            r.is_some_and(|(entry, _)| (entry - 0.431_416).abs() < 1e-4),
            "{r:?}"
        );
    }

    #[test]
    fn tetrahedron_holds_its_inside() {
        let q = [Vec3::X, Vec3::Y, Vec3::Z, Vec3::ZERO].map(|p| p - Vec3::splat(0.1));
        assert!(contains_origin(q, 0.0));
        let mirrored = [q[1], q[0], q[2], q[3]];
        assert!(contains_origin(mirrored, 0.0));
        let outside = q.map(|p| p - Vec3::splat(0.5));
        assert!(!contains_origin(outside, 0.0));
    }
}

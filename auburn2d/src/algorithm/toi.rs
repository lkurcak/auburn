//! Casts between any two convex shapes, given their support functions.
//!
//! A shape `a` moving at `vel` overlaps a shape `b` at time `t` when `t * vel` is in the
//! Minkowski difference `b - a`: the offsets by which `a` would have to move to touch `b`.
//! That set is convex, so the times form one interval, where the line through the origin along
//! `vel` crosses it. [`shape_cast`] finds both ends of it with a GJK ray cast (G. van den
//! Bergen, "Ray Casting against General Convex Objects with Application to Continuous
//! Collision Detection", 2004), one from each side.
//!
//! Its tolerances are relative to the shapes, so it works the same for tiny and huge ones.

use super::gjk::gjk;
use crate::Vec2;
use crate::relation::cast::{direction_and_speed, is_at_rest};

const GJK_MAX_ITERATIONS: usize = 30;
const RAY_CAST_MAX_ITERATIONS: usize = 64;
/// A ray cast hits once the point it has reached is no further from the set than this fraction
/// of the set's size around it.
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
/// use auburn2d::Vec2;
/// use auburn2d::algorithm::shape_cast;
///
/// // A unit ball at the origin, moving right at 2, and a unit ball at (10, 0).
/// let a = |dir: Vec2| dir.normalize_or_zero();
/// let b = |dir: Vec2| Vec2::new(10.0, 0.0) + dir.normalize_or_zero();
/// let (entry, exit) = shape_cast(a, b, Vec2::new(2.0, 0.0)).unwrap();
/// assert!((entry - 4.0).abs() < 1e-4 && (exit - 6.0).abs() < 1e-4);
/// ```
pub fn shape_cast<F1, F2>(support_a: F1, support_b: F2, vel: Vec2) -> Option<(f32, f32)>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    if is_at_rest(vel) {
        let overlap = gjk(&support_a, &support_b, GJK_MAX_ITERATIONS);
        return overlap.map(|_| (f32::NEG_INFINITY, f32::INFINITY));
    }
    let support = |dir: Vec2| support_b(dir) - support_a(-dir);
    let (dir, speed) = direction_and_speed(vel);
    // Along `dir`, the set spans [lo, hi], so the line can only be in it there.
    let lo = support(-dir).dot(dir);
    let hi = support(dir).dot(dir);
    let entry = first_along(support, dir, lo, hi)?;
    // Rounding can make the other side miss a graze, which enters and exits at once.
    let exit = first_along(support, -dir, -hi, -lo).map_or(entry, |t| -t);
    Some((entry / speed, exit.max(entry) / speed))
}

/// The least `t` in [`from`, `to`] at which `t * dir` is in the convex set that `support`
/// describes, or `None` if there is none. `dir` is a unit vector, and the set lies beyond
/// `from` along it: `from` is at most the least `p.dot(dir)` of a point `p` in it.
fn first_along(support: impl Fn(Vec2) -> Vec2, dir: Vec2, from: f32, to: f32) -> Option<f32> {
    let mut t = from;
    let mut x = t * dir;
    let mut simplex = Simplex::new(support(-dir));
    // From `x` to the point of the simplex nearest to it.
    let mut near = simplex.nearest(x);
    for _ in 0..RAY_CAST_MAX_ITERATIONS {
        if near.length_squared() <= simplex.tolerance_squared(x, RELATIVE_TOLERANCE) {
            return Some(t);
        }
        let p = support(-near);
        // All of the set is at least this far along `near` from `x`, times its length.
        let gap = (p - x).dot(near);
        let before = (t, near.length_squared());
        if gap > 0.0 {
            // The line through `p` across `near` separates `x` from the set, so the ray
            // either misses or crosses it before reaching the set.
            let approach = dir.dot(near);
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
        if t == before.0 && near.length_squared() >= before.1 {
            break;
        }
    }
    let close = near.length_squared() <= simplex.tolerance_squared(x, STALL_TOLERANCE);
    close.then_some(t)
}

/// Up to three points of a convex set, the fewest whose hull holds the point of it nearest to
/// the ray's current point.
struct Simplex {
    points: [Vec2; 3],
    len: usize,
}

impl Simplex {
    fn new(p: Vec2) -> Self {
        Self {
            points: [p; 3],
            len: 1,
        }
    }

    fn push(&mut self, p: Vec2) {
        // `nearest` keeps at most two points unless the hull holds `x`, which ends the cast.
        debug_assert!(self.len < 3);
        self.points[self.len] = p;
        self.len += 1;
    }

    /// The squared distance from `x` within which the simplex counts as reached: `relative`
    /// of its size around `x`, or a few units of rounding in its coordinates.
    fn tolerance_squared(&self, x: Vec2, relative: f32) -> f32 {
        let points = &self.points[..self.len];
        let size = points.iter().map(|&p| (p - x).length_squared());
        let magnitude = points.iter().map(|p| p.length_squared());
        let size = size.fold(0.0, f32::max);
        let magnitude = magnitude.fold(x.length_squared(), f32::max);
        (relative * relative * size).max(ROUNDING_TOLERANCE * ROUNDING_TOLERANCE * magnitude)
    }

    /// The vector from `x` to the point of the simplex's hull nearest to it. Drops the points
    /// not needed to hold that point, unless the hull holds `x`, where it returns zero.
    fn nearest(&mut self, x: Vec2) -> Vec2 {
        let q = self.points.map(|p| p - x);
        match self.len {
            1 => q[0],
            2 => self.nearest_on_edge(q, 0, 1),
            _ => {
                let area = (q[1] - q[0]).perp_dot(q[2] - q[0]);
                let inside = area != 0.0
                    && [(0, 1), (1, 2), (2, 0)].iter().all(|&(i, j)| {
                        let side = (q[j] - q[i]).perp_dot(-q[i]);
                        side == 0.0 || (side > 0.0) == (area > 0.0)
                    });
                if inside {
                    return Vec2::ZERO;
                }
                let edges = [(0, 1), (1, 2), (2, 0)].map(|(i, j)| {
                    let mut edge = Simplex {
                        points: self.points,
                        len: 2,
                    };
                    let near = edge.nearest_on_edge(q, i, j);
                    (near, edge)
                });
                let (near, edge) = edges
                    .into_iter()
                    .min_by(|a, b| a.0.length_squared().total_cmp(&b.0.length_squared()))
                    .expect("a triangle has edges");
                *self = edge;
                near
            }
        }
    }

    /// The point nearest to the origin on the edge from `q[i]` to `q[j]`, the simplex's points
    /// seen from `x`. Keeps only that edge's points, or the one vertex if the point is there.
    fn nearest_on_edge(&mut self, q: [Vec2; 3], i: usize, j: usize) -> Vec2 {
        let (a, b) = (q[i], q[j]);
        let edge = b - a;
        let (pa, pb) = (self.points[i], self.points[j]);
        if edge.dot(-a) <= 0.0 {
            *self = Simplex::new(pa);
            a
        } else if edge.dot(-b) >= 0.0 {
            *self = Simplex::new(pb);
            b
        } else {
            // Across the edge, which keeps it accurate when the edge is long.
            self.points[..2].copy_from_slice(&[pa, pb]);
            self.len = 2;
            let normal = edge.perp();
            normal * (normal.dot(a) / normal.length_squared())
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn ball(centre: Vec2, radius: f32) -> impl Fn(Vec2) -> Vec2 {
        move |dir: Vec2| centre + dir.normalize_or_zero() * radius
    }

    fn square(centre: Vec2, half: f32) -> impl Fn(Vec2) -> Vec2 {
        move |dir: Vec2| centre + Vec2::new(half.copysign(dir.x), half.copysign(dir.y))
    }

    fn near(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
        got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-4 && (b - want.1).abs() < 1e-4)
    }

    #[test]
    fn balls_meet_and_part() {
        let a = ball(Vec2::ZERO, 1.0);
        let b = ball(Vec2::new(3.0, 0.0), 1.0);
        let got = shape_cast(a, b, Vec2::X);
        assert!(near(got, (1.0, 5.0)), "{got:?}");
    }

    #[test]
    fn balls_miss() {
        let a = ball(Vec2::ZERO, 1.0);
        let b = ball(Vec2::new(3.0, 3.0), 1.0);
        assert_eq!(shape_cast(a, b, Vec2::X), None);
    }

    #[test]
    fn overlap_now_started_in_the_past() {
        let a = ball(Vec2::ZERO, 1.0);
        let b = ball(Vec2::new(0.5, 0.0), 1.0);
        let got = shape_cast(a, b, Vec2::X);
        assert!(near(got, (-1.5, 2.5)), "{got:?}");
    }

    #[test]
    fn far_beyond_time_one() {
        let a = ball(Vec2::ZERO, 1.0);
        let b = square(Vec2::new(10.0, 0.0), 1.0);
        let got = shape_cast(a, b, Vec2::X);
        assert!(near(got, (8.0, 12.0)), "{got:?}");
    }

    #[test]
    fn ball_meets_corner() {
        // The ball meets the square's corner (2, 0.5) after travelling 2 - √0.75, and leaves
        // its corner (4, 0.5) after travelling 4 + √0.75.
        let a = ball(Vec2::ZERO, 1.0);
        let b = square(Vec2::new(3.0, 1.5), 1.0);
        let got = shape_cast(a, b, Vec2::new(2.0, 0.0));
        let s = 0.75f32.sqrt();
        assert!(near(got, ((2.0 - s) / 2.0, (4.0 + s) / 2.0)), "{got:?}");
    }

    #[test]
    fn same_at_any_scale() {
        for k in [1e-4, 1.0, 1e4] {
            let a = ball(Vec2::ZERO, k);
            let b = square(Vec2::new(3.0, 1.5) * k, k);
            let (entry, exit) = shape_cast(a, b, Vec2::new(2.0, 0.0) * k).unwrap();
            let s = 0.75f32.sqrt();
            let want = ((2.0 - s) / 2.0, (4.0 + s) / 2.0);
            assert!(
                (entry - want.0).abs() < 1e-4 && (exit - want.1).abs() < 1e-4,
                "{k}: ({entry}, {exit}), want {want:?}"
            );
        }
    }

    #[test]
    fn at_rest() {
        let a = ball(Vec2::ZERO, 1.0);
        let b = ball(Vec2::new(1.5, 0.0), 1.0);
        let c = ball(Vec2::new(3.0, 0.0), 1.0);
        let all_time = Some((f32::NEG_INFINITY, f32::INFINITY));
        assert_eq!(shape_cast(&a, b, Vec2::ZERO), all_time);
        assert_eq!(shape_cast(&a, c, Vec2::ZERO), None);
    }
}

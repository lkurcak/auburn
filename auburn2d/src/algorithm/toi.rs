//! Casts between any two convex shapes, given their support functions.
//!
//! A shape `a` moving at `vel` overlaps a shape `b` at time `t` when `t * vel` is in the
//! Minkowski difference `b - a`: the offsets by which `a` would have to move to touch `b`.
//! That set is convex, so the times form one interval, where the line through the origin along
//! `vel` crosses it. [`shape_cast`] finds both ends of it with a GJK ray cast (G. van den
//! Bergen, "Ray Casting against General Convex Objects with Application to Continuous
//! Collision Detection", 2004), one from each side.
//!
//! The pairs with a ball cast it as its centre and its radius. The set is then the other
//! shape's, rounded by that radius: the cast resolves a polygon exactly and steps onto the
//! rounding, and its distances are never shorter than the radius, so rounding errors in the
//! support points don't swamp them. Where it meets the rounding, it settles on the polygon's
//! nearest point, so the normal and the point where the shapes touch are exact too.
//!
//! Its tolerances are relative to the shapes, so it works the same for tiny and huge ones.

use super::gjk::gjk;
use crate::Vec2;
use crate::relation::cast::{direction_and_speed, is_at_rest, unit};

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
    rounded_shape_cast(support_a, support_b, Radii::NONE, vel)
}

/// How far shapes `a` and `b` are rounded: each is a core, which its support function
/// describes, and the points within its radius of that. A ball is a point rounded by its
/// radius.
#[derive(Clone, Copy, Debug)]
pub(crate) struct Radii {
    pub a: f32,
    pub b: f32,
}

impl Radii {
    pub const NONE: Self = Self { a: 0.0, b: 0.0 };

    fn sum(self) -> f32 {
        self.a + self.b
    }
}

/// [`shape_cast`] for shapes rounded by `radii`, with `support_a` and `support_b` their cores.
pub(crate) fn rounded_shape_cast<F1, F2>(
    support_a: F1,
    support_b: F2,
    radii: Radii,
    vel: Vec2,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    if is_at_rest(vel) {
        let radius = radii.sum();
        let rounded_b = |dir: Vec2| support_b(dir) + radius * dir.normalize_or_zero();
        let overlap = gjk(&support_a, rounded_b, GJK_MAX_ITERATIONS);
        return overlap.map(|_| (f32::NEG_INFINITY, f32::INFINITY));
    }
    let (dir, speed) = direction_and_speed(vel);
    let (entry, exit) = sweep(support_a, support_b, radii, dir)?;
    Some((entry.t / speed, exit.t / speed))
}

/// Where a ray reaches the Minkowski difference of a cast: see [`sweep`].
#[derive(Clone, Copy, Debug)]
pub(crate) struct RayHit {
    /// How far along the ray.
    pub t: f32,
    /// The set's outward normal there, a unit vector: the way to move `a` to part the shapes.
    pub normal: Vec2,
    /// A point where the shapes touch then: a point of `b`.
    pub point: Vec2,
}

/// Where shape `a`, moving along the unit vector `dir`, starts and stops overlapping shape
/// `b`, over all of time, in units of distance along `dir`: `None` if never.
///
/// `support_a` and `support_b` are as for [`rounded_shape_cast`].
pub(crate) fn sweep<F1, F2>(
    support_a: F1,
    support_b: F2,
    radii: Radii,
    dir: Vec2,
) -> Option<(RayHit, RayHit)>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    let support = |dir: Vec2| Point::of(&support_a, &support_b, dir);
    let radius = radii.sum();
    // Along `dir`, the set spans [lo, hi], so the line can only be in it there.
    let lo = support(-dir).at.dot(dir) - radius;
    let hi = support(dir).at.dot(dir) + radius;
    let cast = |dir, from, to| first_along(support, radii, dir, from, to, hi - lo);
    let entry = cast(dir, lo, hi)?;
    // Rounding can make the other side miss a graze, which enters and exits at once.
    let exit = cast(-dir, -hi, -lo).map_or(entry, |hit| RayHit { t: -hit.t, ..hit });
    let exit = RayHit {
        t: exit.t.max(entry.t),
        ..exit
    };
    Some((entry, exit))
}

/// The point of the cores' Minkowski difference `b - a` nearest to the origin, as a point of
/// the set and the point of `b`'s core it comes from, or `None` if the set holds the origin.
///
/// `support_a` and `support_b` are as for [`rounded_shape_cast`].
pub(crate) fn nearest_to_origin<F1, F2>(support_a: F1, support_b: F2) -> Option<(Vec2, Vec2)>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    let support = |dir: Vec2| Point::of(&support_a, &support_b, dir);
    let mut simplex = Simplex::new(support(Vec2::X));
    let near = simplex.settle(support, Vec2::ZERO);
    (near != Vec2::ZERO).then(|| (near, simplex.witness(Vec2::ZERO)))
}

/// A point of a Minkowski difference `b - a`, and the point of `b` it comes from.
#[derive(Clone, Copy, Debug)]
struct Point {
    at: Vec2,
    on_b: Vec2,
}

impl Point {
    /// The difference's support point along `dir`.
    fn of(support_a: impl Fn(Vec2) -> Vec2, support_b: impl Fn(Vec2) -> Vec2, dir: Vec2) -> Self {
        let on_b = support_b(dir);
        Self {
            at: on_b - support_a(-dir),
            on_b,
        }
    }
}

/// Where the ray `t * dir` first comes within `radius` of the convex set that `support`
/// describes, for `t` in [`from`, `to`], or `None` if it doesn't. `dir` is a unit vector, and
/// the rounded set lies beyond `from` along it: `from` is at most the least `p.dot(dir)` of a
/// point `p` in it. `extent` is the rounded set's length along `dir`.
fn first_along(
    support: impl Fn(Vec2) -> Point,
    radii: Radii,
    dir: Vec2,
    from: f32,
    to: f32,
    extent: f32,
) -> Option<RayHit> {
    let radius = radii.sum();
    let mut t = from;
    let mut x = t * dir;
    let mut simplex = Simplex::new(support(-dir));
    // Without rounding, the last line found to separate `x` from the set holds `x` once the
    // ray reaches it, so its normal is the set's there. At first, it is the line across `dir`
    // at `from`. With rounding, the normal is out from the set's nearest point: the ray stops
    // as soon as it is close enough to the set, which it can be with that point far along a
    // side from where it should be, so the simplex settles on it first.
    let mut normal = -dir;
    let hit = |mut simplex: Simplex, t: f32, x: Vec2, normal: Vec2| {
        let normal = if radius > 0.0 {
            -simplex.settle(&support, x)
        } else {
            normal
        };
        let normal = unit(normal).unwrap_or(-dir);
        RayHit {
            t,
            normal,
            point: simplex.witness(x) + normal * radii.b,
        }
    };
    // From `x` to the point of the simplex nearest to it.
    let mut near = simplex.nearest(x);
    for _ in 0..RAY_CAST_MAX_ITERATIONS {
        let distance = near.length();
        // `x` is at most this far outside the rounded set, along `near`.
        let excess = distance - radius;
        let scale = simplex.size(x).max(extent);
        if excess <= simplex.rounding(x, radius) {
            return Some(hit(simplex, t, x, normal));
        }
        let towards = near / distance;
        // Met at an angle, a ray still has `excess / approach` to go to reach the set.
        let approach = dir.dot(towards);
        if excess <= RELATIVE_TOLERANCE * scale * approach {
            return Some(hit(simplex, t, x, normal));
        }
        let p = support(-near);
        // All of the rounded set is at least this far along `towards` from `x`.
        let gap = (p.at - x).dot(towards) - radius;
        let before = (t, distance);
        if gap > 0.0 {
            // The line `gap` along `towards` separates `x` from the set, so the ray either
            // misses or crosses it before reaching the set.
            if approach <= 0.0 {
                return None;
            }
            t += gap / approach;
            if t > to {
                return None;
            }
            x = t * dir;
            normal = -near;
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
    (excess <= STALL_TOLERANCE * scale).then(|| hit(simplex, t, x, normal))
}

/// Up to three points of a convex set, the fewest whose hull holds the point of it nearest to
/// the ray's current point, and the points of `b` they come from.
struct Simplex {
    points: [Vec2; 3],
    on_b: [Vec2; 3],
    len: usize,
}

impl Simplex {
    fn new(p: Point) -> Self {
        Self {
            points: [p.at; 3],
            on_b: [p.on_b; 3],
            len: 1,
        }
    }

    fn push(&mut self, p: Point) {
        // `nearest` keeps at most two points unless the hull holds `x`, which ends the cast.
        debug_assert!(self.len < 3);
        self.points[self.len] = p.at;
        self.on_b[self.len] = p.on_b;
        self.len += 1;
    }

    /// The point of `b` that the simplex's point nearest to `x` comes from: the same
    /// combination of the points of `b` as that point is of the simplex's points.
    ///
    /// A triangle holds `x` only once the ray has reached the set's boundary, so `x` is as
    /// close to one of its edges, and that edge's point nearest to `x` stands in for it. Its
    /// own combination could be far off: two of its points can come from one point of `b` and
    /// lie almost on top of each other.
    fn witness(&self, x: Vec2) -> Vec2 {
        let edge = |i: usize, j: usize| {
            let (p, q) = (self.points[i], self.points[j]);
            let (edge, length_squared) = (q - p, (q - p).length_squared());
            let s = if length_squared > 0.0 {
                ((x - p).dot(edge) / length_squared).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let gap = (p + edge * s - x).length_squared();
            (gap, self.on_b[i] + (self.on_b[j] - self.on_b[i]) * s)
        };
        match self.len {
            1 => self.on_b[0],
            2 => edge(0, 1).1,
            _ => {
                [edge(0, 1), edge(1, 2), edge(2, 0)]
                    .into_iter()
                    .min_by(|a, b| a.0.total_cmp(&b.0))
                    .expect("a triangle has edges")
                    .1
            }
        }
    }

    /// Brings the simplex to the set's point nearest to `x` by GJK, and gives the vector from
    /// `x` to it: zero if the set holds `x`. It stops once no point of the set is nearer, as
    /// far as rounding lets it tell: for a polygon, once the support point it finds is one it
    /// has.
    fn settle(&mut self, support: impl Fn(Vec2) -> Point, x: Vec2) -> Vec2 {
        let mut near = self.nearest(x);
        for _ in 0..GJK_MAX_ITERATIONS {
            if near == Vec2::ZERO {
                break;
            }
            let p = support(-near);
            let known = self.points[..self.len].contains(&p.at);
            // No point of the set is nearer than its support point along `near` lets it be.
            if known || (p.at - x).dot(near) >= near.length_squared() {
                break;
            }
            self.push(p);
            let before = near.length_squared();
            near = self.nearest(x);
            if near.length_squared() >= before {
                break;
            }
        }
        near
    }

    /// How far the simplex's points are from `x`, at most.
    fn size(&self, x: Vec2) -> f32 {
        let points = &self.points[..self.len];
        points.iter().map(|&p| p.distance(x)).fold(0.0, f32::max)
    }

    /// How far from `x` rounding alone can put the simplex's nearest point: a few units of f32
    /// precision in its coordinates, in `x`'s and in `radius`.
    fn rounding(&self, x: Vec2, radius: f32) -> f32 {
        let points = &self.points[..self.len];
        let magnitude = points.iter().map(|p| p.length());
        let magnitude = magnitude.fold(x.length().max(radius), f32::max);
        ROUNDING_TOLERANCE * magnitude
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
                        on_b: self.on_b,
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
        let (wa, wb) = (self.on_b[i], self.on_b[j]);
        if edge.dot(-a) <= 0.0 {
            *self = Simplex::new(Point { at: pa, on_b: wa });
            a
        } else if edge.dot(-b) >= 0.0 {
            *self = Simplex::new(Point { at: pb, on_b: wb });
            b
        } else {
            // Across the edge, which keeps it accurate when the edge is long.
            self.points[..2].copy_from_slice(&[pa, pb]);
            self.on_b[..2].copy_from_slice(&[wa, wb]);
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

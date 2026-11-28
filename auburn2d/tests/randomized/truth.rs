//! Exact ground truth, computed in f64 without the library: no `Support`, no GJK.
//!
//! Every shape is a convex vertex set in world space, inflated by a radius. The Minkowski
//! difference of two such shapes is the convex hull of all vertex differences, inflated by the
//! sum of the radii. That makes the signed depth exact: the penetration depth when positive,
//! minus the distance when negative.

use glam::DVec2;

/// A shape in world space: the convex hull of `points`, inflated by `radius`.
#[derive(Clone, Debug)]
pub struct Geometry {
    pub points: Vec<DVec2>,
    pub radius: f64,
}

/// Two shapes, ready to answer depth and sweep questions about moving the first one.
pub struct Truth {
    /// Counter-clockwise convex hull of the vertex differences `a - b`.
    hull: Vec<DVec2>,
    /// Outward unit normal of the edge from `hull[i]` to `hull[i + 1]`.
    normals: Vec<DVec2>,
    radius: f64,
    /// Bounds the Minkowski difference: no point of it is farther from the origin.
    extent: f64,
}

/// What happens when the first shape moves along a line, over all of time.
pub struct Sweep {
    /// The deepest overlap along the line; negative when the shapes never touch.
    pub max_depth: f64,
    /// When the shapes overlap, as `(entry, exit)`.
    pub overlap: Option<(f64, f64)>,
}

impl Truth {
    pub fn new(a: &Geometry, b: &Geometry) -> Self {
        let differences = a
            .points
            .iter()
            .flat_map(|&p| b.points.iter().map(move |&q| p - q))
            .collect();
        let hull = convex_hull(differences);
        let normals = (0..hull.len())
            .map(|i| {
                let edge = hull[(i + 1) % hull.len()] - hull[i];
                DVec2::new(edge.y, -edge.x).normalize_or_zero()
            })
            .collect();
        let extent = hull.iter().map(|p| p.length()).fold(0.0, f64::max) + a.radius + b.radius;
        Self {
            hull,
            normals,
            radius: a.radius + b.radius,
            extent,
        }
    }

    /// Signed depth after translating the first shape by `offset`.
    pub fn depth(&self, offset: DVec2) -> f64 {
        self.radius - self.signed_distance(-offset)
    }

    /// How far apart the line across the unit vector `n` keeps the shapes, once the first is
    /// translated by `offset`: negative if it can't, by as much as the line cuts into them.
    /// The first shape is on the side `n` points to.
    pub fn gap(&self, n: DVec2, offset: DVec2) -> f64 {
        // The least of `p.dot(n)` over the Minkowski difference, which the first shape's
        // translation adds to.
        let least = self
            .hull
            .iter()
            .map(|p| p.dot(n))
            .fold(f64::INFINITY, f64::min);
        least - self.radius + offset.dot(n)
    }

    /// Signed distance from `x` to the hull; negative inside.
    fn signed_distance(&self, x: DVec2) -> f64 {
        let h = &self.hull;
        match h.len() {
            1 => (x - h[0]).length(),
            2 => segment_distance(x, h[0], h[1]),
            n => {
                let outside = (0..n)
                    .map(|i| self.normals[i].dot(x - h[i]))
                    .fold(f64::NEG_INFINITY, f64::max);
                if outside <= 0.0 {
                    outside
                } else {
                    (0..n)
                        .map(|i| segment_distance(x, h[i], h[(i + 1) % n]))
                        .fold(f64::INFINITY, f64::min)
                }
            }
        }
    }

    /// Moves the first shape with velocity `vel` over all of time.
    ///
    /// The depth is concave in time, so a ternary search finds the deepest point and two
    /// bisections find where the depth crosses zero on either side of it.
    pub fn sweep(&self, vel: DVec2) -> Sweep {
        let depth_at = |t: f64| self.depth(vel * t);
        let horizon = 4.0 * self.extent.max(1e-30) / vel.length();
        let (mut lo, mut hi) = (-horizon, horizon);
        for _ in 0..100 {
            let m1 = lo + (hi - lo) / 3.0;
            let m2 = hi - (hi - lo) / 3.0;
            if depth_at(m1) < depth_at(m2) {
                lo = m1;
            } else {
                hi = m2;
            }
        }
        let deepest = 0.5 * (lo + hi);
        let max_depth = depth_at(deepest);
        if max_depth < 0.0 {
            return Sweep {
                max_depth,
                overlap: None,
            };
        }
        let crossing = |mut outside: f64, mut inside: f64| {
            for _ in 0..64 {
                let mid = 0.5 * (outside + inside);
                if depth_at(mid) >= 0.0 {
                    inside = mid;
                } else {
                    outside = mid;
                }
            }
            0.5 * (outside + inside)
        };
        Sweep {
            max_depth,
            overlap: Some((crossing(-horizon, deepest), crossing(horizon, deepest))),
        }
    }
}

/// How far `p` is inside `shape`: negative outside it.
pub fn depth_of(p: DVec2, shape: &Geometry) -> f64 {
    let point = Geometry {
        points: vec![p],
        radius: 0.0,
    };
    Truth::new(&point, shape).depth(DVec2::ZERO)
}

/// Where touching shapes touch, if that is a single point, as it is when either is a point or a
/// ball: a single point with a radius.
pub fn contact(a: &Geometry, b: &Geometry) -> Option<DVec2> {
    // From the point of `core`'s hull nearest to `centre`, out towards it by `radius`.
    let towards = |core: &Geometry, centre: DVec2, radius: f64| {
        let q = nearest_on_hull(&core.points, centre);
        q + (centre - q).normalize_or_zero() * radius
    };
    match (a.points.as_slice(), b.points.as_slice()) {
        (_, &[centre]) => Some(towards(b, nearest_on_hull(&a.points, centre), b.radius)),
        (&[centre], _) => Some(towards(b, centre, b.radius)),
        _ => None,
    }
}

/// The point of the convex polygon through `points` nearest to `x`, which is outside it or on
/// its boundary.
fn nearest_on_hull(points: &[DVec2], x: DVec2) -> DVec2 {
    let n = points.len();
    (0..n)
        .map(|i| nearest_on_segment(x, points[i], points[(i + 1) % n]))
        .min_by(|p, q| (*p - x).length().total_cmp(&(*q - x).length()))
        .expect("a shape has points")
}

fn nearest_on_segment(x: DVec2, a: DVec2, b: DVec2) -> DVec2 {
    let ab = b - a;
    if ab == DVec2::ZERO {
        return a;
    }
    a + ab * ((x - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
}

fn cross(o: DVec2, a: DVec2, b: DVec2) -> f64 {
    (a - o).perp_dot(b - o)
}

/// Andrew's monotone chain; counter-clockwise, without collinear points.
fn convex_hull(mut points: Vec<DVec2>) -> Vec<DVec2> {
    points.sort_by(|a, b| a.x.total_cmp(&b.x).then(a.y.total_cmp(&b.y)));
    points.dedup();
    if points.len() <= 2 {
        return points;
    }
    let mut hull: Vec<DVec2> = Vec::with_capacity(points.len() + 1);
    // The lower chain left to right, then the upper chain right to left.
    let push = |hull: &mut Vec<DVec2>, floor: usize, p: DVec2| {
        while hull.len() >= floor + 2 && cross(hull[hull.len() - 2], hull[hull.len() - 1], p) <= 0.0
        {
            hull.pop();
        }
        hull.push(p);
    };
    for &p in &points {
        push(&mut hull, 0, p);
    }
    let floor = hull.len() - 1;
    for &p in points.iter().rev().skip(1) {
        push(&mut hull, floor, p);
    }
    hull.pop();
    hull
}

fn segment_distance(x: DVec2, a: DVec2, b: DVec2) -> f64 {
    let ab = b - a;
    let t = if ab == DVec2::ZERO {
        0.0
    } else {
        ((x - a).dot(ab) / ab.length_squared()).clamp(0.0, 1.0)
    };
    (x - (a + ab * t)).length()
}

#[cfg(test)]
mod tests {
    use super::*;

    fn square(center: DVec2, half: f64) -> Geometry {
        let points = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)]
            .iter()
            .map(|&(x, y)| center + DVec2::new(x, y) * half)
            .collect();
        Geometry {
            points,
            radius: 0.0,
        }
    }

    fn ball(center: DVec2, radius: f64) -> Geometry {
        Geometry {
            points: vec![center],
            radius,
        }
    }

    #[test]
    fn depth_of_known_pairs() {
        let a = ball(DVec2::ZERO, 1.0);
        let b = ball(DVec2::new(3.0, 0.0), 1.0);
        assert!((Truth::new(&a, &b).depth(DVec2::ZERO) + 1.0).abs() < 1e-12);
        let a = square(DVec2::ZERO, 1.0);
        let b = square(DVec2::new(1.5, 0.25), 1.0);
        assert!((Truth::new(&a, &b).depth(DVec2::ZERO) - 0.5).abs() < 1e-12);
        let corner = Truth::new(&square(DVec2::ZERO, 1.0), &ball(DVec2::new(2.0, 2.0), 0.5));
        assert!((corner.depth(DVec2::ZERO) - (0.5 - 2f64.sqrt())).abs() < 1e-12);
    }

    #[test]
    fn sweep_of_known_pair() {
        let a = ball(DVec2::ZERO, 1.0);
        let b = square(DVec2::new(5.0, 0.0), 1.0);
        let sweep = Truth::new(&a, &b).sweep(DVec2::new(2.0, 0.0));
        let (entry, exit) = sweep.overlap.unwrap();
        assert!((entry - 1.5).abs() < 1e-9 && (exit - 3.5).abs() < 1e-9);
        assert!((sweep.max_depth - 2.0).abs() < 1e-9);
    }
}

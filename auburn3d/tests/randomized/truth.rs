//! Exact ground truth, computed in f64 without the library: no `Support`, no GJK.
//!
//! Spheres (and points, as spheres of radius 0) and oriented boxes have closed forms:
//! sphere against sphere by the distance between centres, sphere against box in the box's
//! frame, and box against box by the 15 axes of the separating axis theorem. The last is exact
//! for overlap depth and for whether the boxes overlap; for separated boxes it gives no more
//! than the distance, which keeps "too close to call" conservative.

use glam::DVec3;

/// A shape in world space.
#[derive(Clone, Debug)]
pub enum Geometry {
    Sphere {
        center: DVec3,
        radius: f64,
    },
    Box {
        center: DVec3,
        /// Unit axes of the box's frame.
        axes: [DVec3; 3],
        half: [f64; 3],
    },
}

/// Two shapes, ready to answer depth and sweep questions about moving the first one.
pub struct Truth {
    a: Geometry,
    b: Geometry,
}

/// What happens when the first shape moves along a line, over all of time.
pub struct Sweep {
    /// The deepest overlap along the line; negative when the shapes never touch.
    pub max_depth: f64,
    /// When the shapes overlap, as `(entry, exit)`.
    pub overlap: Option<(f64, f64)>,
}

impl Geometry {
    fn center(&self) -> DVec3 {
        match self {
            Geometry::Sphere { center, .. } | Geometry::Box { center, .. } => *center,
        }
    }

    fn reach(&self) -> f64 {
        match self {
            Geometry::Sphere { radius, .. } => *radius,
            Geometry::Box { half, .. } => DVec3::from(*half).length(),
        }
    }
}

/// Signed distance from `p` to a box; negative inside.
fn box_distance(p: DVec3, center: DVec3, axes: &[DVec3; 3], half: &[f64; 3]) -> f64 {
    let d: [f64; 3] = std::array::from_fn(|i| (p - center).dot(axes[i]).abs() - half[i]);
    let outside = DVec3::from(d).max(DVec3::ZERO).length();
    let inside = d[0].max(d[1]).max(d[2]).min(0.0);
    outside + inside
}

/// The smallest overlap of two boxes over the separating axes; negative when one separates.
fn box_box_depth(a: (DVec3, &[DVec3; 3], &[f64; 3]), b: (DVec3, &[DVec3; 3], &[f64; 3])) -> f64 {
    let ((ca, axa, ha), (cb, axb, hb)) = (a, b);
    let mut axes: Vec<DVec3> = axa.iter().chain(axb).copied().collect();
    for u in axa {
        for v in axb {
            let c = u.cross(*v);
            // Parallel edges add nothing the face axes don't already cover.
            if c.length() > 1e-9 {
                axes.push(c.normalize());
            }
        }
    }
    let radius = |axes: &[DVec3; 3], half: &[f64; 3], l: DVec3| -> f64 {
        (0..3).map(|i| half[i] * axes[i].dot(l).abs()).sum()
    };
    axes.iter()
        .map(|&l| radius(axa, ha, l) + radius(axb, hb, l) - (cb - ca).dot(l).abs())
        .fold(f64::INFINITY, f64::min)
}

impl Truth {
    pub fn new(a: &Geometry, b: &Geometry) -> Self {
        Self {
            a: a.clone(),
            b: b.clone(),
        }
    }

    /// Signed depth after translating the first shape by `offset`.
    pub fn depth(&self, offset: DVec3) -> f64 {
        use Geometry::{Box, Sphere};
        match (&self.a, &self.b) {
            (
                Sphere {
                    center: c1,
                    radius: r1,
                },
                Sphere {
                    center: c2,
                    radius: r2,
                },
            ) => r1 + r2 - (*c1 + offset - *c2).length(),
            (
                Sphere { center, radius },
                Box {
                    center: cb,
                    axes,
                    half,
                },
            ) => radius - box_distance(*center + offset, *cb, axes, half),
            (
                Box {
                    center: cb,
                    axes,
                    half,
                },
                Sphere { center, radius },
            ) => radius - box_distance(*center - offset, *cb, axes, half),
            (
                Box {
                    center: ca,
                    axes: aa,
                    half: ha,
                },
                Box {
                    center: cb,
                    axes: ab,
                    half: hb,
                },
            ) => box_box_depth((*ca + offset, aa, ha), (*cb, ab, hb)),
        }
    }

    /// Moves the first shape with velocity `vel` over all of time.
    ///
    /// The depth is concave in time, so a ternary search finds the deepest point and two
    /// bisections find where the depth crosses zero on either side of it.
    pub fn sweep(&self, vel: DVec3) -> Sweep {
        let depth_at = |t: f64| self.depth(vel * t);
        let extent = (self.a.center() - self.b.center()).length() + self.a.reach() + self.b.reach();
        let horizon = 4.0 * extent.max(1e-30) / vel.length();
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

#[cfg(test)]
mod tests {
    use super::*;

    fn cube(center: DVec3, half: f64) -> Geometry {
        Geometry::Box {
            center,
            axes: [DVec3::X, DVec3::Y, DVec3::Z],
            half: [half; 3],
        }
    }

    #[test]
    fn depth_of_known_pairs() {
        let ball = |center, radius| Geometry::Sphere { center, radius };
        let t = Truth::new(
            &ball(DVec3::ZERO, 1.0),
            &ball(DVec3::new(3.0, 0.0, 0.0), 1.0),
        );
        assert!((t.depth(DVec3::ZERO) + 1.0).abs() < 1e-12);
        let t = Truth::new(
            &cube(DVec3::ZERO, 1.0),
            &cube(DVec3::new(1.5, 0.25, 0.0), 1.0),
        );
        assert!((t.depth(DVec3::ZERO) - 0.5).abs() < 1e-12);
        let t = Truth::new(
            &ball(DVec3::new(2.0, 2.0, 2.0), 0.5),
            &cube(DVec3::ZERO, 1.0),
        );
        assert!((t.depth(DVec3::ZERO) - (0.5 - 3f64.sqrt())).abs() < 1e-12);
        // Rotated 45° about z, the cube reaches √2 along x.
        let h = 0.5f64.sqrt();
        let diamond = Geometry::Box {
            center: DVec3::new(3.0, 0.0, 0.0),
            axes: [DVec3::new(h, h, 0.0), DVec3::new(-h, h, 0.0), DVec3::Z],
            half: [1.0; 3],
        };
        let t = Truth::new(&cube(DVec3::ZERO, 1.0), &diamond);
        assert!((t.depth(DVec3::ZERO) - (2f64.sqrt() - 2.0)).abs() < 1e-12);
    }

    #[test]
    fn sweep_of_known_pair() {
        let ball = Geometry::Sphere {
            center: DVec3::ZERO,
            radius: 1.0,
        };
        let t = Truth::new(&ball, &cube(DVec3::new(5.0, 0.0, 0.0), 1.0));
        let sweep = t.sweep(DVec3::new(2.0, 0.0, 0.0));
        let (entry, exit) = sweep.overlap.unwrap();
        assert!((entry - 1.5).abs() < 1e-9 && (exit - 3.5).abs() < 1e-9);
        assert!((sweep.max_depth - 2.0).abs() < 1e-9);
    }
}

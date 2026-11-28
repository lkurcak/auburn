//! The 3D queries as a game would write them by hand, for the pairs with a short formula.
//!
//! As in 2D, each pair comes down to a point against a shape at the origin: the moving shape's
//! centre against the other shape grown by it. Two balls are a point against a sphere of both
//! radii, two boxes a point against a box of both half sizes, and a ball against a box a point
//! against a rounded box. A turned box does the same in its own frame.
//!
//! A ball cast against a box has no short formula, as the rounded box has edges as well as
//! corners, and neither do two boxes turned different ways.
//!
//! In every pair, `a` moves at `vel` and `b` stands still.

use auburn3d::{Quat, Vec3};

/// A shape as a game would keep it: where it is, how it's turned, and how big it is.
pub struct Body {
    pub pos: Vec3,
    pub rot: Quat,
    /// The radius, if it is a ball.
    pub radius: f32,
    /// The half size, if it is a box.
    pub half_size: Vec3,
}

/// A sphere of `radius` at the origin, and a point at `p` moving at `v`.
pub mod sphere {
    use super::Vec3;

    pub fn contains(p: Vec3, radius: f32) -> bool {
        p.length_squared() <= radius * radius
    }

    pub fn push(p: Vec3, radius: f32) -> Option<Vec3> {
        let squared = p.length_squared();
        if squared > radius * radius {
            return None;
        }
        let d = squared.sqrt();
        Some(if d > 0.0 {
            p * ((radius - d) / d)
        } else {
            Vec3::new(radius, 0.0, 0.0)
        })
    }

    pub fn toi(p: Vec3, radius: f32, v: Vec3) -> Option<f32> {
        let c = p.length_squared() - radius * radius;
        if c <= 0.0 {
            return Some(0.0);
        }
        let b = p.dot(v);
        if b >= 0.0 {
            return None;
        }
        let discriminant = b * b - v.length_squared() * c;
        if discriminant < 0.0 {
            return None;
        }
        Some(c / (-b + discriminant.sqrt()))
    }
}

/// A box of `half` size at the origin, and a point at `p` moving at `v`: the slab test.
pub mod aabb {
    use super::Vec3;

    pub fn contains(p: Vec3, half: Vec3) -> bool {
        p.abs().cmple(half).all()
    }

    pub fn push(p: Vec3, half: Vec3) -> Option<Vec3> {
        let depth = half - p.abs();
        if depth.min_element() < 0.0 {
            return None;
        }
        Some(if depth.x < depth.y && depth.x < depth.z {
            Vec3::new(depth.x.copysign(p.x), 0.0, 0.0)
        } else if depth.y < depth.z {
            Vec3::new(0.0, depth.y.copysign(p.y), 0.0)
        } else {
            Vec3::new(0.0, 0.0, depth.z.copysign(p.z))
        })
    }

    pub fn toi(p: Vec3, half: Vec3, v: Vec3) -> Option<f32> {
        let inv = v.recip();
        let (t1, t2) = ((-half - p) * inv, (half - p) * inv);
        let (entry, exit) = (t1.min(t2).max_element(), t1.max(t2).min_element());
        (entry <= exit && exit >= 0.0).then_some(entry.max(0.0))
    }
}

/// A box of `half` size at the origin rounded by `radius`, and a point at `p`.
pub mod rounded {
    use super::Vec3;

    pub fn contains(p: Vec3, half: Vec3, radius: f32) -> bool {
        (p.abs() - half).max(Vec3::ZERO).length_squared() <= radius * radius
    }

    pub fn push(p: Vec3, half: Vec3, radius: f32) -> Option<Vec3> {
        let out = p.abs() - half;
        if out.max_element() <= 0.0 {
            // The centre is in the box: out across the nearest side.
            return Some(if out.x > out.y && out.x > out.z {
                Vec3::new((radius - out.x).copysign(p.x), 0.0, 0.0)
            } else if out.y > out.z {
                Vec3::new(0.0, (radius - out.y).copysign(p.y), 0.0)
            } else {
                Vec3::new(0.0, 0.0, (radius - out.z).copysign(p.z))
            });
        }
        let out = out.max(Vec3::ZERO);
        let squared = out.length_squared();
        if squared > radius * radius {
            return None;
        }
        let d = squared.sqrt();
        Some(out.copysign(p) * ((radius - d) / d))
    }
}

pub fn point_ball_collides(a: &Body, b: &Body) -> bool {
    sphere::contains(a.pos - b.pos, b.radius)
}

pub fn point_ball_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    sphere::push(a.pos - b.pos, b.radius)
}

pub fn point_ball_toi(a: &Body, b: &Body, vel: Vec3) -> Option<f32> {
    sphere::toi(a.pos - b.pos, b.radius, vel)
}

pub fn ball_ball_collides(a: &Body, b: &Body) -> bool {
    sphere::contains(a.pos - b.pos, a.radius + b.radius)
}

pub fn ball_ball_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    sphere::push(a.pos - b.pos, a.radius + b.radius)
}

pub fn ball_ball_toi(a: &Body, b: &Body, vel: Vec3) -> Option<f32> {
    sphere::toi(a.pos - b.pos, a.radius + b.radius, vel)
}

pub fn point_box_collides(a: &Body, b: &Body) -> bool {
    aabb::contains(a.pos - b.pos, b.half_size)
}

pub fn point_box_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    aabb::push(a.pos - b.pos, b.half_size)
}

pub fn point_box_toi(a: &Body, b: &Body, vel: Vec3) -> Option<f32> {
    aabb::toi(a.pos - b.pos, b.half_size, vel)
}

pub fn ball_box_collides(a: &Body, b: &Body) -> bool {
    rounded::contains(a.pos - b.pos, b.half_size, a.radius)
}

pub fn ball_box_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    rounded::push(a.pos - b.pos, b.half_size, a.radius)
}

pub fn box_box_collides(a: &Body, b: &Body) -> bool {
    aabb::contains(a.pos - b.pos, a.half_size + b.half_size)
}

pub fn box_box_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    aabb::push(a.pos - b.pos, a.half_size + b.half_size)
}

pub fn box_box_toi(a: &Body, b: &Body, vel: Vec3) -> Option<f32> {
    aabb::toi(a.pos - b.pos, a.half_size + b.half_size, vel)
}

pub fn point_turned_box_collides(a: &Body, b: &Body) -> bool {
    aabb::contains(b.rot.inverse() * (a.pos - b.pos), b.half_size)
}

pub fn point_turned_box_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    aabb::push(b.rot.inverse() * (a.pos - b.pos), b.half_size).map(|push| b.rot * push)
}

pub fn point_turned_box_toi(a: &Body, b: &Body, vel: Vec3) -> Option<f32> {
    let to_local = b.rot.inverse();
    aabb::toi(to_local * (a.pos - b.pos), b.half_size, to_local * vel)
}

pub fn ball_turned_box_collides(a: &Body, b: &Body) -> bool {
    rounded::contains(b.rot.inverse() * (a.pos - b.pos), b.half_size, a.radius)
}

pub fn ball_turned_box_penetrates(a: &Body, b: &Body) -> Option<Vec3> {
    let push = rounded::push(b.rot.inverse() * (a.pos - b.pos), b.half_size, a.radius)?;
    Some(b.rot * push)
}

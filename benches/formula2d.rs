//! The 2D queries as a game would write them by hand, for the pairs with a short formula.
//!
//! Each pair comes down to a point against a shape at the origin: the moving shape's centre
//! against the other shape grown by it. Two balls are a point against a circle of both radii,
//! two boxes a point against a box of both half sizes, and a ball against a box a point
//! against a rounded box. A turned box does the same in its own frame. Two boxes turned
//! different ways have no short formula: that takes the separating axis test.
//!
//! In every pair, `a` moves at `vel` and `b` stands still.

use auburn2d::Vec2;
use auburn2d::prelude::CastHit;

/// A shape as a game would keep it: where it is, how it's turned, and how big it is.
pub struct Body {
    pub pos: Vec2,
    pub angle: f32,
    /// The same rotation as its cosine and sine, which the formulas use.
    pub rot: Vec2,
    /// The radius, if it is a ball.
    pub radius: f32,
    /// The half size, if it is a box.
    pub half_size: Vec2,
}

/// `v` in the frame of something turned by `rot`.
fn to_local(v: Vec2, rot: Vec2) -> Vec2 {
    Vec2::new(rot.x, -rot.y).rotate(v)
}

/// The time, the normal out of the shape, and how deep the point is in it then: 0 unless it
/// is inside from the start.
type Entry = (f32, Vec2, f32);

/// A circle of `radius` at the origin, and a point at `p` moving at `v`.
pub mod circle {
    use super::{Entry, Vec2};

    pub fn contains(p: Vec2, radius: f32) -> bool {
        p.length_squared() <= radius * radius
    }

    pub fn push(p: Vec2, radius: f32) -> Option<Vec2> {
        let squared = p.length_squared();
        if squared > radius * radius {
            return None;
        }
        let d = squared.sqrt();
        Some(if d > 0.0 {
            p * ((radius - d) / d)
        } else {
            Vec2::new(radius, 0.0)
        })
    }

    pub fn toi(p: Vec2, radius: f32, v: Vec2) -> Option<f32> {
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

    pub fn entry(p: Vec2, radius: f32, v: Vec2) -> Option<Entry> {
        let t = toi(p, radius, v)?;
        let q = p + v * t;
        let d = q.length();
        let normal = if d > 0.0 { q / d } else { Vec2::X };
        Some((t, normal, (radius - d).max(0.0)))
    }
}

/// A box of `half` size at the origin, and a point at `p` moving at `v`: the slab test.
pub mod aabb {
    use super::{Entry, Vec2};

    pub fn contains(p: Vec2, half: Vec2) -> bool {
        p.abs().cmple(half).all()
    }

    pub fn push(p: Vec2, half: Vec2) -> Option<Vec2> {
        let depth = half - p.abs();
        if depth.min_element() < 0.0 {
            return None;
        }
        Some(if depth.x < depth.y {
            Vec2::new(depth.x.copysign(p.x), 0.0)
        } else {
            Vec2::new(0.0, depth.y.copysign(p.y))
        })
    }

    pub fn toi(p: Vec2, half: Vec2, v: Vec2) -> Option<f32> {
        let inv = v.recip();
        let (t1, t2) = ((-half - p) * inv, (half - p) * inv);
        let (entry, exit) = (t1.min(t2).max_element(), t1.max(t2).min_element());
        (entry <= exit && exit >= 0.0).then_some(entry.max(0.0))
    }

    pub fn entry(p: Vec2, half: Vec2, v: Vec2) -> Option<Entry> {
        let inv = v.recip();
        let (t1, t2) = ((-half - p) * inv, (half - p) * inv);
        let near = t1.min(t2);
        let (entry, exit) = (near.max_element(), t1.max(t2).min_element());
        if entry > exit || exit < 0.0 {
            return None;
        }
        if entry < 0.0 {
            // Inside already: out across the nearest side.
            let depth = half - p.abs();
            return Some(if depth.x < depth.y {
                (0.0, Vec2::new(1f32.copysign(p.x), 0.0), depth.x)
            } else {
                (0.0, Vec2::new(0.0, 1f32.copysign(p.y)), depth.y)
            });
        }
        let normal = if near.x > near.y {
            Vec2::new(-v.x.signum(), 0.0)
        } else {
            Vec2::new(0.0, -v.y.signum())
        };
        Some((entry, normal, 0.0))
    }
}

/// A box of `half` size at the origin rounded by `radius`, and a point at `p` moving at `v`:
/// the box grown by the radius, and a circle at the corner if the point gets there.
pub mod rounded {
    use super::{Entry, Vec2, aabb, circle};

    pub fn contains(p: Vec2, half: Vec2, radius: f32) -> bool {
        (p.abs() - half).max(Vec2::ZERO).length_squared() <= radius * radius
    }

    pub fn push(p: Vec2, half: Vec2, radius: f32) -> Option<Vec2> {
        let out = p.abs() - half;
        if out.x <= 0.0 && out.y <= 0.0 {
            // The centre is in the box: out across the nearest side.
            return Some(if out.x > out.y {
                Vec2::new((radius - out.x).copysign(p.x), 0.0)
            } else {
                Vec2::new(0.0, (radius - out.y).copysign(p.y))
            });
        }
        let out = out.max(Vec2::ZERO);
        let squared = out.length_squared();
        if squared > radius * radius {
            return None;
        }
        let d = squared.sqrt();
        Some(out.copysign(p) * ((radius - d) / d))
    }

    pub fn toi(p: Vec2, half: Vec2, radius: f32, v: Vec2) -> Option<f32> {
        let t = aabb::toi(p, half + radius, v)?;
        let q = p + v * t;
        let out = q.abs() - half;
        if out.x <= 0.0 || out.y <= 0.0 {
            return Some(t);
        }
        circle::toi(p - half.copysign(q), radius, v)
    }

    pub fn entry(p: Vec2, half: Vec2, radius: f32, v: Vec2) -> Option<Entry> {
        let entry = aabb::entry(p, half + radius, v)?;
        let q = p + v * entry.0;
        let out = q.abs() - half;
        if out.x <= 0.0 || out.y <= 0.0 {
            return Some(entry);
        }
        circle::entry(p - half.copysign(q), radius, v)
    }
}

/// `a`'s centre where it first touches `b`, from an entry relative to `b`'s centre: moved out
/// along the normal if it starts inside.
fn hit((time, normal, depth): Entry, a: &Body, vel: Vec2) -> (f32, Vec2, Vec2) {
    (time, normal, a.pos + vel * time + normal * depth)
}

pub fn point_ball_collides(a: &Body, b: &Body) -> bool {
    circle::contains(a.pos - b.pos, b.radius)
}

pub fn point_ball_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    circle::push(a.pos - b.pos, b.radius)
}

pub fn point_ball_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    circle::toi(a.pos - b.pos, b.radius, vel)
}

pub fn point_ball_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let (time, normal, point) = hit(circle::entry(a.pos - b.pos, b.radius, vel)?, a, vel);
    Some(CastHit {
        time,
        normal,
        point,
    })
}

pub fn ball_ball_collides(a: &Body, b: &Body) -> bool {
    circle::contains(a.pos - b.pos, a.radius + b.radius)
}

pub fn ball_ball_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    circle::push(a.pos - b.pos, a.radius + b.radius)
}

pub fn ball_ball_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    circle::toi(a.pos - b.pos, a.radius + b.radius, vel)
}

pub fn ball_ball_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let entry = circle::entry(a.pos - b.pos, a.radius + b.radius, vel)?;
    let (time, normal, centre) = hit(entry, a, vel);
    Some(CastHit {
        time,
        normal,
        point: centre - normal * a.radius,
    })
}

pub fn point_box_collides(a: &Body, b: &Body) -> bool {
    aabb::contains(a.pos - b.pos, b.half_size)
}

pub fn point_box_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    aabb::push(a.pos - b.pos, b.half_size)
}

pub fn point_box_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    aabb::toi(a.pos - b.pos, b.half_size, vel)
}

pub fn point_box_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let (time, normal, point) = hit(aabb::entry(a.pos - b.pos, b.half_size, vel)?, a, vel);
    Some(CastHit {
        time,
        normal,
        point,
    })
}

pub fn ball_box_collides(a: &Body, b: &Body) -> bool {
    rounded::contains(a.pos - b.pos, b.half_size, a.radius)
}

pub fn ball_box_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    rounded::push(a.pos - b.pos, b.half_size, a.radius)
}

pub fn ball_box_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    rounded::toi(a.pos - b.pos, b.half_size, a.radius, vel)
}

pub fn ball_box_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let entry = rounded::entry(a.pos - b.pos, b.half_size, a.radius, vel)?;
    let (time, normal, centre) = hit(entry, a, vel);
    Some(CastHit {
        time,
        normal,
        point: centre - normal * a.radius,
    })
}

pub fn box_box_collides(a: &Body, b: &Body) -> bool {
    aabb::contains(a.pos - b.pos, a.half_size + b.half_size)
}

pub fn box_box_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    aabb::push(a.pos - b.pos, a.half_size + b.half_size)
}

pub fn box_box_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    aabb::toi(a.pos - b.pos, a.half_size + b.half_size, vel)
}

pub fn box_box_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let entry = aabb::entry(a.pos - b.pos, a.half_size + b.half_size, vel)?;
    let (time, normal, centre) = hit(entry, a, vel);
    // The middle of where the boxes meet.
    let lo = (centre - a.half_size).max(b.pos - b.half_size);
    let hi = (centre + a.half_size).min(b.pos + b.half_size);
    Some(CastHit {
        time,
        normal,
        point: 0.5 * (lo + hi),
    })
}

/// A point or a ball against a turned box, in the box's frame: `a`'s position and velocity
/// there, and `b` as if it were not turned.
fn in_frame_of(a: &Body, b: &Body, vel: Vec2) -> (Body, Body, Vec2) {
    let at = |pos| Body {
        pos,
        angle: 0.0,
        rot: Vec2::X,
        radius: a.radius,
        half_size: a.half_size,
    };
    let local = at(to_local(a.pos - b.pos, b.rot));
    let origin = Body {
        pos: Vec2::ZERO,
        angle: 0.0,
        rot: Vec2::X,
        radius: b.radius,
        half_size: b.half_size,
    };
    (local, origin, to_local(vel, b.rot))
}

/// A hit found in `b`'s frame, back in the world.
fn to_world(hit: CastHit, b: &Body) -> CastHit {
    CastHit {
        normal: b.rot.rotate(hit.normal),
        point: b.pos + b.rot.rotate(hit.point),
        ..hit
    }
}

pub fn point_turned_box_collides(a: &Body, b: &Body) -> bool {
    aabb::contains(to_local(a.pos - b.pos, b.rot), b.half_size)
}

pub fn point_turned_box_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    aabb::push(to_local(a.pos - b.pos, b.rot), b.half_size).map(|push| b.rot.rotate(push))
}

pub fn point_turned_box_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    let (a, b, vel) = in_frame_of(a, b, vel);
    point_box_toi(&a, &b, vel)
}

pub fn point_turned_box_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let (local_a, local_b, local_vel) = in_frame_of(a, b, vel);
    point_box_cast(&local_a, &local_b, local_vel).map(|hit| to_world(hit, b))
}

pub fn ball_turned_box_collides(a: &Body, b: &Body) -> bool {
    rounded::contains(to_local(a.pos - b.pos, b.rot), b.half_size, a.radius)
}

pub fn ball_turned_box_penetrates(a: &Body, b: &Body) -> Option<Vec2> {
    let push = rounded::push(to_local(a.pos - b.pos, b.rot), b.half_size, a.radius)?;
    Some(b.rot.rotate(push))
}

pub fn ball_turned_box_toi(a: &Body, b: &Body, vel: Vec2) -> Option<f32> {
    let (a, b, vel) = in_frame_of(a, b, vel);
    ball_box_toi(&a, &b, vel)
}

pub fn ball_turned_box_cast(a: &Body, b: &Body, vel: Vec2) -> Option<CastHit> {
    let (local_a, local_b, local_vel) = in_frame_of(a, b, vel);
    ball_box_cast(&local_a, &local_b, local_vel).map(|hit| to_world(hit, b))
}

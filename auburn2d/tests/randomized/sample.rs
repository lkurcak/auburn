//! Seeded inputs: a small PRNG plus generators for shapes, transformations and velocities.
//!
//! The generators use only `+`, `-`, `*`, `/` and `sqrt`, which IEEE 754 rounds the same way
//! everywhere, so every platform sees the same cases.

use crate::truth::Geometry;
use auburn2d::Quat;
use auburn2d::prelude::*;
use auburn2d::transformation::pos2d::Pos2d;
use glam::DVec2;
use std::cmp::Ordering;
use std::fmt::Debug;
use std::sync::Arc;

/// xorshift64*.
pub struct Rng(u64);

impl Rng {
    /// Seeds from a label, so each check draws the same cases however the tests are grouped.
    pub fn from_label(label: &str) -> Self {
        let fnv1a = label.bytes().fold(0xcbf2_9ce4_8422_2325_u64, |h, b| {
            (h ^ u64::from(b)).wrapping_mul(0x0100_0000_01b3)
        });
        Self(fnv1a | 1)
    }

    fn next(&mut self) -> u64 {
        self.0 ^= self.0 >> 12;
        self.0 ^= self.0 << 25;
        self.0 ^= self.0 >> 27;
        self.0.wrapping_mul(0x2545_f491_4f6c_dd1d)
    }

    /// Uniform in `[0, 1)`.
    pub fn unit(&mut self) -> f32 {
        (self.next() >> 40) as f32 / (1u64 << 24) as f32
    }

    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        lo + (hi - lo) * self.unit()
    }

    pub fn below(&mut self, n: usize) -> usize {
        (self.next() % n as u64) as usize
    }

    pub fn chance(&mut self, p: f32) -> bool {
        self.unit() < p
    }

    pub fn vec2(&mut self, lo: f32, hi: f32) -> Vec2 {
        Vec2::new(self.range(lo, hi), self.range(lo, hi))
    }

    /// A uniformly distributed unit vector, by rejection sampling.
    pub fn direction(&mut self) -> Vec2 {
        loop {
            let v = self.vec2(-1.0, 1.0);
            let len2 = v.length_squared();
            if (0.01..=1.0).contains(&len2) {
                return v / len2.sqrt();
            }
        }
    }

    /// `0.0` or `-0.0`, which take different paths in some casts.
    pub fn zero(&mut self) -> f32 {
        if self.chance(0.5) { 0.0 } else { -0.0 }
    }
}

#[derive(Clone, Copy, Debug)]
pub enum Scale {
    One,
    Positive,
    /// Positive or negative.
    Mirrored,
}

/// The kind of scene a check samples.
#[derive(Clone, Copy, Debug)]
pub struct Config {
    pub name: &'static str,
    /// Positions are uniform in `[-spread, spread]²`, shifted by `offset`.
    pub spread: f32,
    pub offset: Vec2,
    /// Radii and half extents are uniform in `[min_size, max_size]`.
    pub min_size: f32,
    pub max_size: f32,
    pub max_speed: f32,
    /// Whether `Transform2d` and bevy `Transform` rotate.
    pub rotate: bool,
    /// How `Transform2d`, `ScaleTranslate2d` and bevy `Transform` scale.
    pub scale: Scale,
}

impl Config {
    /// Rotation and positive scale, at sizes around 1.
    pub const BASE: Self = Self {
        name: "base",
        spread: 3.0,
        offset: Vec2::ZERO,
        min_size: 0.1,
        max_size: 2.0,
        max_speed: 10.0,
        rotate: true,
        scale: Scale::Positive,
    };
    /// Rotation, no scale.
    pub const RIGID: Self = Self {
        name: "rigid",
        scale: Scale::One,
        ..Self::BASE
    };
    /// Neither rotation nor scale.
    pub const AXIS_ALIGNED: Self = Self {
        name: "axis-aligned",
        rotate: false,
        scale: Scale::One,
        ..Self::BASE
    };
    /// Rotation and scale of either sign.
    pub const MIRRORED: Self = Self {
        name: "mirrored",
        scale: Scale::Mirrored,
        ..Self::BASE
    };
    /// Everything 10⁴ times smaller.
    pub const TINY: Self = Self::BASE.scaled("tiny", 1e-4);
    /// Everything 10⁴ times larger.
    pub const HUGE: Self = Self::BASE.scaled("huge", 1e4);
    /// Sizes around 1, about 10⁴ away from the origin.
    pub const FAR: Self = Self {
        name: "far",
        offset: Vec2::new(1e4, -1e4),
        ..Self::BASE
    };

    const fn scaled(self, name: &'static str, k: f32) -> Self {
        Self {
            name,
            spread: self.spread * k,
            min_size: self.min_size * k,
            max_size: self.max_size * k,
            max_speed: self.max_speed * k,
            ..self
        }
    }

    /// Cases whose true depth is within this distance of zero are too close to call: the
    /// library may answer either way. It also bounds the error allowed in vectors and times.
    pub fn tolerance(&self) -> f64 {
        let far = f64::from(self.offset.length() + 3.0 * self.spread);
        1e-3 * f64::from(self.max_size) + far * 16.0 * f64::from(f32::EPSILON)
    }

    fn position(&self, rng: &mut Rng) -> Vec2 {
        rng.vec2(-self.spread, self.spread) + self.offset
    }

    fn scale(&self, rng: &mut Rng) -> f32 {
        let magnitude = match self.scale {
            Scale::One => return 1.0,
            _ if rng.chance(0.3) => 1.0,
            _ => rng.range(0.3, 3.0),
        };
        match self.scale {
            Scale::Mirrored if rng.chance(0.5) => -magnitude,
            _ => magnitude,
        }
    }

    /// `(cos, sin)` of a random angle, or of a multiple of 90° a quarter of the time.
    fn rotation(&self, rng: &mut Rng) -> Vec2 {
        if !self.rotate {
            return Vec2::X;
        }
        if rng.chance(0.25) {
            [Vec2::X, Vec2::Y, Vec2::NEG_X, Vec2::NEG_Y][rng.below(4)]
        } else {
            rng.direction()
        }
    }
}

/// `f32` to `f64`, exactly.
pub fn wide(v: Vec2) -> DVec2 {
    DVec2::new(f64::from(v.x), f64::from(v.y))
}

/// A transformation the checks can sample and evaluate independently of the library.
pub trait Place: Transformation2d + Clone + Debug {
    const NAME: &'static str;
    fn sample(rng: &mut Rng, cfg: &Config) -> Self;
    /// Where the local point `p` ends up, computed in f64 from the fields.
    fn world(&self, p: Vec2) -> DVec2;
    /// The factor by which the transformation scales lengths.
    fn length_scale(&self) -> f64;
}

impl Place for Vec2 {
    const NAME: &'static str = "Vec2";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        cfg.position(rng)
    }
    fn world(&self, p: Vec2) -> DVec2 {
        wide(p) + wide(*self)
    }
    fn length_scale(&self) -> f64 {
        1.0
    }
}

impl Place for Pos2d {
    const NAME: &'static str = "Pos2d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        Pos2d(cfg.position(rng))
    }
    fn world(&self, p: Vec2) -> DVec2 {
        wide(p) + wide(self.0)
    }
    fn length_scale(&self) -> f64 {
        1.0
    }
}

impl Place for ScaleTranslate2d {
    const NAME: &'static str = "ScaleTranslate2d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let translation = cfg.position(rng);
        ScaleTranslate2d::new(translation, cfg.scale(rng))
    }
    fn world(&self, p: Vec2) -> DVec2 {
        wide(p) * f64::from(self.scale) + wide(self.translation)
    }
    fn length_scale(&self) -> f64 {
        f64::from(self.scale.abs())
    }
}

/// The rotation about z with the given `(cos, sin)`, as a half-angle quaternion, so no
/// trigonometry is involved.
fn z_rotation(cos_sin: Vec2) -> Quat {
    let w = ((1.0 + cos_sin.x) / 2.0).max(0.0).sqrt();
    if w > 0.1 {
        Quat::from_xyzw(0.0, 0.0, cos_sin.y / (2.0 * w), w)
    } else {
        // Near 180°, w is tiny, so take z from the cosine instead.
        let z = ((1.0 - cos_sin.x) / 2.0).max(0.0).sqrt();
        Quat::from_xyzw(0.0, 0.0, z, cos_sin.y / (2.0 * z))
    }
}

fn rotate((c, s): (f64, f64), p: DVec2) -> DVec2 {
    DVec2::new(c * p.x - s * p.y, s * p.x + c * p.y)
}

impl Place for Transform2d {
    const NAME: &'static str = "Transform2d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let pos = cfg.position(rng);
        let rot = Rotor2d::from_quaternion(z_rotation(cfg.rotation(rng)));
        let scale = cfg.scale(rng);
        Transform2d { pos, rot, scale }
    }
    fn world(&self, p: Vec2) -> DVec2 {
        let (c, s) = self.rot.to_cos_sin();
        let p = wide(p) * f64::from(self.scale);
        rotate((f64::from(c), f64::from(s)), p) + wide(self.pos)
    }
    fn length_scale(&self) -> f64 {
        f64::from(self.scale.abs())
    }
}

#[cfg(feature = "bevy")]
impl Place for bevy::prelude::Transform {
    const NAME: &'static str = "bevy Transform";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let pos = cfg.position(rng);
        let rotation = z_rotation(cfg.rotation(rng));
        let scale = cfg.scale(rng);
        bevy::prelude::Transform {
            translation: pos.extend(0.0),
            rotation,
            scale: auburn2d::Vec3::splat(scale),
        }
    }
    fn world(&self, p: Vec2) -> DVec2 {
        let (w, z) = (f64::from(self.rotation.w), f64::from(self.rotation.z));
        let p = wide(p) * f64::from(self.scale.x);
        let norm = w * w + z * z;
        let cos_sin = ((w * w - z * z) / norm, 2.0 * w * z / norm);
        rotate(cos_sin, p) + wide(self.translation.truncate())
    }
    fn length_scale(&self) -> f64 {
        f64::from(self.scale.x.abs())
    }
}

/// A shape the checks can sample and describe exactly.
pub trait Sample: Shape + Debug + Sized {
    const NAME: &'static str;
    fn sample(rng: &mut Rng, cfg: &Config) -> Self;
    fn geometry(&self, place: &impl Place) -> Geometry;
}

impl Sample for Point {
    const NAME: &'static str = "Point";
    fn sample(_: &mut Rng, _: &Config) -> Self {
        Point
    }
    fn geometry(&self, place: &impl Place) -> Geometry {
        Geometry {
            points: vec![place.world(Vec2::ZERO)],
            radius: 0.0,
        }
    }
}

impl Sample for Ball {
    const NAME: &'static str = "Ball";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        Ball::new(rng.range(cfg.min_size, cfg.max_size))
    }
    fn geometry(&self, place: &impl Place) -> Geometry {
        Geometry {
            points: vec![place.world(Vec2::ZERO)],
            radius: f64::from(self.radius) * place.length_scale(),
        }
    }
}

impl Sample for Box2d {
    const NAME: &'static str = "Box2d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        Box2d::new(rng.vec2(cfg.min_size, cfg.max_size))
    }
    fn geometry(&self, place: &impl Place) -> Geometry {
        let h = self.half_size;
        let corners = [(1.0, 1.0), (-1.0, 1.0), (-1.0, -1.0), (1.0, -1.0)];
        Geometry {
            points: corners
                .iter()
                .map(|&(x, y)| place.world(Vec2::new(x * h.x, y * h.y)))
                .collect(),
            radius: 0.0,
        }
    }
}

/// Orders directions by angle, counter-clockwise from +x, without `atan2`.
fn by_angle(a: &Vec2, b: &Vec2) -> Ordering {
    let upper = |v: &Vec2| v.y > 0.0 || (v.y == 0.0 && v.x > 0.0);
    upper(b)
        .cmp(&upper(a))
        .then_with(|| 0.0.partial_cmp(&a.perp_dot(*b)).unwrap_or(Ordering::Equal))
}

impl Sample for Poly2d {
    const NAME: &'static str = "Poly2d";
    /// A random convex polygon with 3 to 8 vertices, either way round.
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let directions = loop {
            let n = 3 + rng.below(6);
            let mut d: Vec<Vec2> = (0..n).map(|_| rng.direction()).collect();
            d.sort_by(by_angle);
            // Every turn between neighbours is between about 9° and 171°.
            if (0..n).all(|i| d[i].perp_dot(d[(i + 1) % n]) > 0.15) {
                break d;
            }
        };
        let axes = rng.vec2(cfg.min_size, cfg.max_size);
        let tilt = rng.direction();
        let center = rng.vec2(-0.3 * cfg.max_size, 0.3 * cfg.max_size);
        let mut points: Vec<Vec2> = directions
            .iter()
            .map(|&d| {
                let p = d * axes;
                Vec2::new(tilt.x * p.x - tilt.y * p.y, tilt.y * p.x + tilt.x * p.y) + center
            })
            .collect();
        if rng.chance(0.5) {
            points.reverse();
        }
        Poly2d::new(Arc::new(points))
    }
    fn geometry(&self, place: &impl Place) -> Geometry {
        Geometry {
            points: self.points.iter().map(|&p| place.world(p)).collect(),
            radius: 0.0,
        }
    }
}

impl Sample for Shape2d {
    const NAME: &'static str = "Shape2d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        match rng.below(3) {
            0 => Shape2d::Point(Point),
            1 => Shape2d::Ball(Ball::sample(rng, cfg)),
            _ => Shape2d::Box2d(Box2d::sample(rng, cfg)),
        }
    }
    fn geometry(&self, place: &impl Place) -> Geometry {
        match self {
            Shape2d::Point(s) => s.geometry(place),
            Shape2d::Ball(s) => s.geometry(place),
            Shape2d::Box2d(s) => s.geometry(place),
        }
    }
}

/// A nonzero velocity for a shape at `from`, often roughly toward a shape at `to`, sometimes
/// along an axis.
pub fn velocity(rng: &mut Rng, cfg: &Config, from: Vec2, to: Vec2) -> Vec2 {
    let d = to - from;
    let speed = rng.range(0.2, 1.0) * cfg.max_speed;
    let mut toward = |x: f32| {
        if rng.chance(0.85) {
            x.signum()
        } else {
            -x.signum()
        }
    };
    let (tx, ty) = (toward(d.x), toward(d.y));
    let v = match rng.below(7) {
        0..=2 => d * rng.range(0.2, 3.0) + rng.vec2(-0.5, 0.5) * cfg.max_size,
        3 => rng.vec2(-cfg.max_speed, cfg.max_speed),
        4 => Vec2::new(tx * speed, rng.zero()),
        5 => Vec2::new(rng.zero(), ty * speed),
        _ => Vec2::new(tx, ty) * speed,
    };
    if v.length() < 1e-3 * cfg.max_speed {
        Vec2::new(speed, rng.zero())
    } else {
        v
    }
}

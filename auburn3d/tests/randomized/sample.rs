//! Seeded inputs: a small PRNG plus generators for shapes, transformations and velocities.
//!
//! The generators use only `+`, `-`, `*`, `/` and `sqrt`, which IEEE 754 rounds the same way
//! everywhere, so every platform sees the same cases.

use crate::truth::Geometry;
use auburn3d::Quat;
use auburn3d::prelude::*;
use auburn3d::transformation::pos3d::Pos3d;
use glam::{DQuat, DVec3};
use std::fmt::Debug;

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

    pub fn vec3(&mut self, lo: f32, hi: f32) -> Vec3 {
        Vec3::new(self.range(lo, hi), self.range(lo, hi), self.range(lo, hi))
    }

    /// A uniformly distributed rotation, by rejection sampling in the unit 4-ball.
    pub fn rotation(&mut self) -> Quat {
        loop {
            let q = Quat::from_xyzw(
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
                self.range(-1.0, 1.0),
            );
            let len2 = q.length_squared();
            if (0.01..=1.0).contains(&len2) {
                return q * (1.0 / len2.sqrt());
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
    /// Positions are uniform in `[-spread, spread]³`, shifted by `offset`.
    pub spread: f32,
    pub offset: Vec3,
    /// Radii and half extents are uniform in `[min_size, max_size]`.
    pub min_size: f32,
    pub max_size: f32,
    pub max_speed: f32,
    /// Whether `Transform3d` and bevy `Transform` rotate.
    pub rotate: bool,
    /// How `Transform3d`, `ScaleTranslate3d` and bevy `Transform` scale.
    pub scale: Scale,
}

impl Config {
    /// Rotation and positive scale, at sizes around 1.
    pub const BASE: Self = Self {
        name: "base",
        spread: 3.0,
        offset: Vec3::ZERO,
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
        offset: Vec3::new(1e4, -1e4, 1e4),
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

    fn position(&self, rng: &mut Rng) -> Vec3 {
        rng.vec3(-self.spread, self.spread) + self.offset
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

    /// A random rotation, or a quarter turn about an axis a fifth of the time.
    fn rotation(&self, rng: &mut Rng) -> Quat {
        if !self.rotate {
            return Quat::IDENTITY;
        }
        if rng.chance(0.2) {
            // (sin, cos) of half of 0°, 90°, 180° and 270°.
            let h = 0.5f32.sqrt();
            let (s, c) = [(0.0, 1.0), (h, h), (1.0, 0.0), (h, -h)][rng.below(4)];
            let mut axis = [0.0; 3];
            axis[rng.below(3)] = s;
            Quat::from_xyzw(axis[0], axis[1], axis[2], c)
        } else {
            rng.rotation()
        }
    }
}

/// `f32` to `f64`, exactly.
pub fn wide(v: Vec3) -> DVec3 {
    DVec3::new(f64::from(v.x), f64::from(v.y), f64::from(v.z))
}

/// A transformation the checks can sample and evaluate independently of the library.
pub trait Place: Transformation3d + Clone + Debug {
    const NAME: &'static str;
    fn sample(rng: &mut Rng, cfg: &Config) -> Self;
    /// Where the local origin ends up, computed in f64 from the fields.
    fn origin(&self) -> DVec3;
    /// Where the local axes point, as unit vectors.
    fn axes(&self) -> [DVec3; 3] {
        [DVec3::X, DVec3::Y, DVec3::Z]
    }
    /// The factor by which the transformation scales lengths.
    fn length_scale(&self) -> f64 {
        1.0
    }
}

impl Place for Vec3 {
    const NAME: &'static str = "Vec3";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        cfg.position(rng)
    }
    fn origin(&self) -> DVec3 {
        wide(*self)
    }
}

impl Place for Pos3d {
    const NAME: &'static str = "Pos3d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        Pos3d(cfg.position(rng))
    }
    fn origin(&self) -> DVec3 {
        wide(self.0)
    }
}

impl Place for ScaleTranslate3d {
    const NAME: &'static str = "ScaleTranslate3d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let translation = cfg.position(rng);
        ScaleTranslate3d::new(translation, cfg.scale(rng))
    }
    fn origin(&self) -> DVec3 {
        wide(self.translation)
    }
    fn length_scale(&self) -> f64 {
        f64::from(self.scale.abs())
    }
}

/// The columns of a rotation, in f64. Both shapes are symmetric, so a mirroring scale needs
/// no extra care.
fn columns(q: Quat) -> [DVec3; 3] {
    let q = DQuat::from_xyzw(
        f64::from(q.x),
        f64::from(q.y),
        f64::from(q.z),
        f64::from(q.w),
    )
    .normalize();
    [q * DVec3::X, q * DVec3::Y, q * DVec3::Z]
}

impl Place for Transform3d {
    const NAME: &'static str = "Transform3d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let pos = cfg.position(rng);
        let rot = cfg.rotation(rng);
        let scale = cfg.scale(rng);
        Transform3d { pos, rot, scale }
    }
    fn origin(&self) -> DVec3 {
        wide(self.pos)
    }
    fn axes(&self) -> [DVec3; 3] {
        columns(self.rot)
    }
    fn length_scale(&self) -> f64 {
        f64::from(self.scale.abs())
    }
}

#[cfg(feature = "bevy")]
impl Place for bevy::prelude::Transform {
    const NAME: &'static str = "bevy Transform";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        let translation = cfg.position(rng);
        let rotation = cfg.rotation(rng);
        let scale = cfg.scale(rng);
        bevy::prelude::Transform {
            translation,
            rotation,
            scale: Vec3::splat(scale),
        }
    }
    fn origin(&self) -> DVec3 {
        wide(self.translation)
    }
    fn axes(&self) -> [DVec3; 3] {
        columns(self.rotation)
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
        Geometry::Sphere {
            center: place.origin(),
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
        Geometry::Sphere {
            center: place.origin(),
            radius: f64::from(self.radius) * place.length_scale(),
        }
    }
}

impl Sample for Box3d {
    const NAME: &'static str = "Box3d";
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        Box3d::new(rng.vec3(cfg.min_size, cfg.max_size))
    }
    fn geometry(&self, place: &impl Place) -> Geometry {
        let s = place.length_scale();
        let h = wide(self.half_size) * s;
        Geometry::Box {
            center: place.origin(),
            axes: place.axes(),
            half: [h.x, h.y, h.z],
        }
    }
}

/// A nonzero velocity for a shape at `from`, often roughly toward a shape at `to`, sometimes
/// along an axis.
pub fn velocity(rng: &mut Rng, cfg: &Config, from: Vec3, to: Vec3) -> Vec3 {
    let d = to - from;
    let speed = rng.range(0.2, 1.0) * cfg.max_speed;
    let mut toward = |x: f32| {
        if rng.chance(0.85) {
            x.signum()
        } else {
            -x.signum()
        }
    };
    let t = Vec3::new(toward(d.x), toward(d.y), toward(d.z));
    let v = match rng.below(8) {
        0..=2 => d * rng.range(0.2, 3.0) + rng.vec3(-0.5, 0.5) * cfg.max_size,
        3 => rng.vec3(-cfg.max_speed, cfg.max_speed),
        4 => Vec3::new(t.x * speed, rng.zero(), rng.zero()),
        5 => Vec3::new(rng.zero(), t.y * speed, rng.zero()),
        6 => Vec3::new(rng.zero(), rng.zero(), t.z * speed),
        _ => t * speed,
    };
    if v.length() < 1e-3 * cfg.max_speed {
        Vec3::new(speed, rng.zero(), rng.zero())
    } else {
        v
    }
}

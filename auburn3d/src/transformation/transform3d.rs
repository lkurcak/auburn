use super::*;
use crate::{Quat, Vec3};
use std::ops::Mul;

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// Apply translation, rotation, and scale in this order:
/// 1. Scale
/// 2. Rotate
/// 3. Translate
pub struct Transform3d {
    pub pos: Vec3,
    pub rot: Quat,
    pub scale: f32,
}

impl Default for Transform3d {
    fn default() -> Self {
        Self {
            pos: Vec3::ZERO,
            rot: Quat::IDENTITY,
            scale: 1.0,
        }
    }
}

impl Transformation3d for Transform3d {
    fn apply_to_origin(&self) -> Vec3 {
        self.pos
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.rot * (self.scale * point) + self.pos
    }

    fn inverse(&self) -> Self {
        let inv_rot = self.rot.inverse();
        Self {
            pos: inv_rot * (Vec3::ZERO - self.pos) / self.scale,
            rot: inv_rot,
            scale: 1.0 / self.scale,
        }
    }

    fn compose(&self, other: &Self) -> Self {
        // Standard convention: a.compose(&b) means apply b first, then a.
        Self {
            pos: self.rot * (self.scale * other.pos) + self.pos,
            rot: self.rot * other.rot,
            scale: self.scale * other.scale,
        }
    }
}

impl Transform3d {
    pub const IDENTITY: Self = Self {
        pos: Vec3::ZERO,
        rot: Quat::IDENTITY,
        scale: 1.0,
    };

    pub const fn from_translation(translation: Vec3) -> Self {
        Self {
            pos: translation,
            rot: Quat::IDENTITY,
            scale: 1.0,
        }
    }

    pub fn from_rotation(rotation: Quat) -> Self {
        Self {
            pos: Vec3::ZERO,
            rot: rotation,
            scale: 1.0,
        }
    }

    pub fn from_scale(scale: f32) -> Self {
        Self {
            pos: Vec3::ZERO,
            rot: Quat::IDENTITY,
            scale,
        }
    }

    pub fn with_translation(mut self, translation: Vec3) -> Self {
        self.pos = translation;
        self
    }

    pub fn with_rotation(mut self, rotation: Quat) -> Self {
        self.rot = rotation;
        self
    }

    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }
}

impl Mul<Transform3d> for Transform3d {
    type Output = Transform3d;

    fn mul(self, rhs: Transform3d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<Vec3> for Transform3d {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        self.apply(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose() {
        let a = Transform3d::from_translation(Vec3::new(1.0, 2.0, 3.0))
            .with_rotation(Quat::from_rotation_y(std::f32::consts::FRAC_PI_2));
        let b = Transform3d::from_translation(Vec3::new(3.0, 4.0, 5.0))
            .with_rotation(Quat::from_rotation_z(std::f32::consts::FRAC_PI_4));
        let composed = a.compose(&b);

        let point = Vec3::new(5.0, 7.0, 9.0);
        let expected = a.apply(b.apply(point));
        let actual = composed.apply(point);
        assert!(
            (expected - actual).length() < 1e-5,
            "compose does not match sequential apply"
        );
    }
}

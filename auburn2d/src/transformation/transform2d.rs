use super::*;
use crate::Vec2;
use std::ops::Mul;

#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// Apply translation, rotation, and scale in this order:
/// 1. Scale
/// 1. Rotate
/// 1. Translate
pub struct Transform2d {
    pub pos: Vec2,
    pub rot: Rotor2d,
    pub scale: f32,
}

impl Default for Transform2d {
    fn default() -> Self {
        Self {
            pos: Vec2::ZERO,
            rot: Rotor2d::IDENTITY,
            scale: 1.0,
        }
    }
}

impl Transformation2d for Transform2d {
    fn apply_to_origin(&self) -> Vec2 {
        self.pos
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        self.rot * (self.scale * point) + self.pos
    }

    fn inverse(&self) -> Self {
        Self {
            pos: (self.rot.inverse() * (Vec2::ZERO - self.pos)) / self.scale,
            rot: self.rot.inverse(),
            scale: 1.0 / self.scale,
        }
    }

    fn compose(&self, other: &Self) -> Self {
        // Standard convention: a.compose(&b) means apply b first, then a.
        // T_a(T_b(x)) = rot_a * (scale_a * (rot_b * (scale_b * x) + pos_b)) + pos_a
        //             = (rot_a * rot_b) * ((scale_a * scale_b) * x) + rot_a * (scale_a * pos_b) + pos_a
        Self {
            pos: self.rot * (self.scale * other.pos) + self.pos,
            rot: self.rot * other.rot,
            scale: self.scale * other.scale,
        }
    }
}

#[cfg(feature = "bevy")]
impl From<Transform2d> for bevy::prelude::Isometry2d {
    fn from(value: Transform2d) -> Self {
        let (cos, sin) = value.rot.to_cos_sin();
        Self {
            rotation: bevy::math::Rot2::from_sin_cos(sin, cos),
            translation: value.pos,
        }
    }
}

impl Transform2d {
    pub const IDENTITY: Self = Self {
        pos: Vec2::ZERO,
        rot: Rotor2d::IDENTITY,
        scale: 1.0,
    };

    pub fn from_translation(translation: Vec2) -> Self {
        Self {
            pos: translation,
            ..Default::default()
        }
    }

    pub fn from_angle(angle: f32) -> Self {
        Self {
            rot: Rotor2d::radians(angle),
            ..Default::default()
        }
    }

    pub fn from_rotation(rotation: Rotor2d) -> Self {
        Self {
            rot: rotation,
            ..Default::default()
        }
    }

    pub fn from_scale(scale: f32) -> Self {
        Self {
            scale,
            ..Default::default()
        }
    }

    pub fn with_translation(mut self, translation: Vec2) -> Self {
        self.pos = translation;
        self
    }

    pub fn with_angle(mut self, angle: f32) -> Self {
        self.rot = Rotor2d::radians(angle);
        self
    }

    pub fn with_rotation(mut self, rotation: Rotor2d) -> Self {
        self.rot = rotation;
        self
    }

    pub fn with_scale(mut self, scale: f32) -> Self {
        self.scale = scale;
        self
    }
}

impl Mul<Transform2d> for Transform2d {
    type Output = Transform2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: Transform2d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<&Transform2d> for Transform2d {
    type Output = Transform2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: &Transform2d) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<Transform2d> for &Transform2d {
    type Output = Transform2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: Transform2d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<&Transform2d> for &Transform2d {
    type Output = Transform2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: &Transform2d) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<Vec2> for Transform2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: Vec2) -> Self::Output {
        self.apply(rhs)
    }
}

impl Mul<&Vec2> for Transform2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self.apply(*rhs)
    }
}

impl Mul<Vec2> for &Transform2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: Vec2) -> Self::Output {
        self.apply(rhs)
    }
}

impl Mul<&Vec2> for &Transform2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self.apply(*rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn compose() {
        let a = Transform2d::from_translation(Vec2::new(1.0, 2.0))
            .with_angle(std::f32::consts::FRAC_PI_2)
            .with_scale(2.0);
        let b = Transform2d::from_translation(Vec2::new(3.0, 4.0))
            .with_angle(std::f32::consts::FRAC_PI_4)
            .with_scale(3.0);
        let composed = a.compose(&b);

        // Verify that apply is consistent with compose:
        // a.compose(&b) means apply b first, then a.
        let point = Vec2::new(5.0, 7.0);
        let expected = a.apply(b.apply(point));
        let actual = composed.apply(point);
        assert!(
            (expected - actual).length() < 1e-5,
            "compose does not match sequential apply: expected {:?}, got {:?}",
            expected,
            actual
        );
    }

    #[test]
    fn mul_consistency() {
        let a = Transform2d::from_translation(Vec2::new(1.0, 2.0))
            .with_angle(std::f32::consts::FRAC_PI_2)
            .with_scale(2.0);
        let b = Transform2d::from_translation(Vec2::new(3.0, 4.0))
            .with_angle(std::f32::consts::FRAC_PI_4)
            .with_scale(3.0);
        let point = Vec2::new(5.0, 7.0);

        // * for compose is consistent with apply
        let composed = a.compose(&b);
        assert_eq!(a.clone() * b.clone(), composed, "a * b != a.compose(&b)");
        assert_eq!(a.clone() * &b, composed, "a * &b != a.compose(&b)");
        assert_eq!(&a * b.clone(), composed, "&a * b != a.compose(&b)");
        assert_eq!(&a * &b, composed, "&a * &b != a.compose(&b)");

        let diff = |x: Vec2, y: Vec2| (x - y).length() < 1e-5;
        assert!(
            diff((a.clone() * b.clone()) * point, a * (b * point)),
            "(a * b) * point != a * (b * point)"
        );
    }
}

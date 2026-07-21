use crate::Vec3;
use std::ops::Mul;

use super::Transformation3d;

#[derive(Debug, Clone, PartialEq)]
/// A transformation representing a uniform scale followed by a translation.
///
/// This transformation is defined by `x' = scale * x + translation`.
pub struct ScaleTranslate3d {
    pub translation: Vec3,
    pub scale: f32,
}

impl Transformation3d for ScaleTranslate3d {
    fn apply_to_origin(&self) -> Vec3 {
        self.translation
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point * self.scale + self.translation
    }

    fn inverse(&self) -> Self {
        Self {
            translation: -self.translation / self.scale,
            scale: 1.0 / self.scale,
        }
    }

    fn compose(&self, other: &Self) -> Self {
        // Standard convention: a.compose(&b) means apply b first, then a.
        Self {
            translation: self.scale * other.translation + self.translation,
            scale: self.scale * other.scale,
        }
    }
}

impl Default for ScaleTranslate3d {
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl ScaleTranslate3d {
    /// The identity transformation (scale 1.0, translation zero).
    pub const IDENTITY: Self = Self {
        translation: Vec3::ZERO,
        scale: 1.0,
    };

    /// Creates a transformation with the given uniform scale and zero translation.
    pub const fn from_scale(scale: f32) -> Self {
        Self {
            translation: Vec3::ZERO,
            scale,
        }
    }

    /// Creates a transformation with the given translation and scale 1.0.
    pub const fn from_translation(translation: Vec3) -> Self {
        Self {
            translation,
            scale: 1.0,
        }
    }

    /// Creates a new transformation with the given translation and scale.
    pub const fn new(translation: Vec3, scale: f32) -> Self {
        Self { translation, scale }
    }
}

impl Mul<ScaleTranslate3d> for ScaleTranslate3d {
    type Output = ScaleTranslate3d;

    fn mul(self, rhs: ScaleTranslate3d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<Vec3> for ScaleTranslate3d {
    type Output = Vec3;

    fn mul(self, rhs: Vec3) -> Self::Output {
        self.apply(rhs)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_to_origin() {
        let isometry = ScaleTranslate3d::from_translation(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(isometry.apply_to_origin(), Vec3::new(1.0, 2.0, 3.0));
    }

    #[test]
    fn apply() {
        let isometry = ScaleTranslate3d::from_translation(Vec3::new(1.0, 2.0, 3.0));
        assert_eq!(isometry.apply(Vec3::ONE), Vec3::new(2.0, 3.0, 4.0));
    }

    #[test]
    fn inverse() {
        let isometry = ScaleTranslate3d::new(Vec3::new(1.0, 2.0, 3.0), 2.0);
        assert_eq!(
            isometry.inverse(),
            ScaleTranslate3d::new(Vec3::new(-0.5, -1.0, -1.5), 0.5)
        );
    }

    #[test]
    fn compose() {
        let a = ScaleTranslate3d::new(Vec3::new(1.0, 2.0, 3.0), 2.0);
        let b = ScaleTranslate3d::new(Vec3::new(3.0, 4.0, 5.0), 3.0);
        let composed = a.compose(&b);
        assert_eq!(composed.scale, 6.0);
        assert_eq!(composed.translation, Vec3::new(7.0, 10.0, 13.0));
    }
}

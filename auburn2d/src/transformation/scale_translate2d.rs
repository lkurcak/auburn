use crate::Vec2;
use std::ops::Mul;

use super::Transformation2d;

#[derive(Debug, Clone, PartialEq)]
/// A transformation representing a uniform scale followed by a translation.
///
/// This transformation is defined by `x' = scale * x + translation`.
///
/// # Examples
///
/// ```
/// use auburn2d::prelude::*;
/// use auburn2d::Vec2;
///
/// let transform = ScaleTranslate2d::new(Vec2::new(10.0, 5.0), 2.0);
/// let point = Vec2::new(1.0, 1.0);
/// let transformed = transform.apply(point);
///
/// assert_eq!(transformed, Vec2::new(12.0, 7.0));
/// ```
pub struct ScaleTranslate2d {
    pub translation: Vec2,
    pub scale: f32,
}

// Note: This Into implementation is incomplete and not currently used.
// Isometry2d in Bevy doesn't support non-uniform scale.
#[cfg(feature = "bevy")]
#[allow(unreachable_code)]
impl Into<bevy::prelude::Isometry2d> for ScaleTranslate2d {
    fn into(self) -> bevy::prelude::Isometry2d {
        bevy::prelude::Isometry2d {
            rotation: todo!(),
            translation: todo!(),
        }
    }
}

impl Transformation2d for ScaleTranslate2d {
    fn apply_to_origin(&self) -> Vec2 {
        self.translation
    }

    fn apply(&self, point: Vec2) -> Vec2 {
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
        // T_a(T_b(x)) = scale_a * (scale_b * x + translation_b) + translation_a
        //             = (scale_a * scale_b) * x + (scale_a * translation_b + translation_a)
        Self {
            translation: self.scale * other.translation + self.translation,
            scale: self.scale * other.scale,
        }
    }
}

impl Mul<ScaleTranslate2d> for ScaleTranslate2d {
    type Output = ScaleTranslate2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: ScaleTranslate2d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<&ScaleTranslate2d> for ScaleTranslate2d {
    type Output = ScaleTranslate2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: &ScaleTranslate2d) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<ScaleTranslate2d> for &ScaleTranslate2d {
    type Output = ScaleTranslate2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: ScaleTranslate2d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<&ScaleTranslate2d> for &ScaleTranslate2d {
    type Output = ScaleTranslate2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: &ScaleTranslate2d) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<Vec2> for ScaleTranslate2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: Vec2) -> Self::Output {
        self.apply(rhs)
    }
}

impl Mul<&Vec2> for ScaleTranslate2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self.apply(*rhs)
    }
}

impl Mul<Vec2> for &ScaleTranslate2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: Vec2) -> Self::Output {
        self.apply(rhs)
    }
}

impl Mul<&Vec2> for &ScaleTranslate2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self.apply(*rhs)
    }
}

impl Default for ScaleTranslate2d {
    /// Returns the identity transformation.
    fn default() -> Self {
        Self::IDENTITY
    }
}

impl ScaleTranslate2d {
    /// The identity transformation (scale 1.0, translation (0, 0)).
    pub const IDENTITY: Self = Self {
        translation: Vec2::ZERO,
        scale: 1.0,
    };

    /// Creates a transformation with the given uniform scale and zero translation.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::Vec2;
    ///
    /// let tr = ScaleTranslate2d::from_scale(2.0);
    /// assert_eq!(tr.scale, 2.0);
    /// assert_eq!(tr.translation, Vec2::ZERO);
    /// ```
    pub fn from_scale(scale: f32) -> Self {
        Self {
            translation: Vec2::ZERO,
            scale,
        }
    }

    /// Creates a transformation with the given translation and scale 1.0.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::Vec2;
    ///
    /// let tr = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0));
    /// assert_eq!(tr.translation, Vec2::new(1.0, 2.0));
    /// assert_eq!(tr.scale, 1.0);
    /// ```
    pub fn from_translation(translation: Vec2) -> Self {
        Self {
            translation,
            scale: 1.0,
        }
    }

    /// Returns a new transformation with the same translation but a new scale.
    pub fn with_scale(&self, scale: f32) -> Self {
        Self {
            translation: self.translation,
            scale,
        }
    }

    /// Returns a new transformation with the same scale but a new translation.
    pub fn with_translation(&self, translation: Vec2) -> Self {
        Self {
            translation,
            scale: self.scale,
        }
    }

    /// Creates a new transformation with the given translation and scale.
    pub fn new(translation: Vec2, scale: f32) -> Self {
        Self { translation, scale }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn apply_to_origin() {
        let isometry = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0));
        assert_eq!(isometry.apply_to_origin(), Vec2::new(1.0, 2.0));

        let isometry = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0)).with_scale(2.0);
        assert_eq!(isometry.apply_to_origin(), Vec2::new(1.0, 2.0));
    }

    #[test]
    fn apply() {
        let isometry = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0));
        assert_eq!(isometry.apply(Vec2::ONE), Vec2::new(2.0, 3.0));

        let isometry = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0)).with_scale(2.0);
        assert_eq!(isometry.apply(Vec2::ONE), Vec2::new(3.0, 4.0));
    }

    #[test]
    fn inverse() {
        let isometry = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0));
        assert_eq!(
            isometry.inverse(),
            ScaleTranslate2d::from_translation(Vec2::new(-1.0, -2.0))
        );

        let isometry = ScaleTranslate2d::from_translation(Vec2::new(1.0, 2.0)).with_scale(2.0);
        assert_eq!(
            isometry.inverse(),
            ScaleTranslate2d::from_translation(Vec2::new(-0.5, -1.0)).with_scale(0.5)
        );
    }

    #[test]
    fn compose() {
        let a = ScaleTranslate2d::new(Vec2::new(1.0, 2.0), 2.0);
        let b = ScaleTranslate2d::new(Vec2::new(3.0, 4.0), 3.0);
        let composed = a.compose(&b);
        assert_eq!(composed.scale, 6.0);
        assert_eq!(composed.translation, Vec2::new(7.0, 10.0));

        // Verify that apply is consistent with compose:
        // a.compose(&b) means apply b first, then a.
        let point = Vec2::new(5.0, 7.0);
        assert_eq!(a.apply(b.apply(point)), composed.apply(point));
    }

    #[test]
    fn mul_consistency() {
        let a = ScaleTranslate2d::new(Vec2::new(1.0, 2.0), 2.0);
        let b = ScaleTranslate2d::new(Vec2::new(3.0, 4.0), 3.0);
        let point = Vec2::new(5.0, 7.0);

        // * for compose is consistent with apply
        assert_eq!(a.clone() * b.clone(), a.compose(&b));
        assert_eq!(a.clone() * &b, a.compose(&b));
        assert_eq!(&a * b.clone(), a.compose(&b));
        assert_eq!(&a * &b, a.compose(&b));
        assert_eq!((a.clone() * b.clone()) * point, a * (b * point));
    }
}

use super::Transformation2d;
use crate::Vec2;
use std::ops::Mul;

#[derive(Default, Debug, Clone, PartialEq)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// Equivalent to [`Vec2`]
pub struct Pos2d(pub Vec2);

impl Pos2d {
    pub fn new(x: f32, y: f32) -> Self {
        Self(Vec2::new(x, y))
    }
}

impl Transformation2d for Pos2d {
    fn apply_to_origin(&self) -> Vec2 {
        self.0.apply_to_origin()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        self.0.apply(point)
    }

    fn inverse(&self) -> Self {
        Self(self.0.inverse())
    }

    fn compose(&self, other: &Self) -> Self {
        Self(self.0.compose(&other.0))
    }
}

impl Mul<Pos2d> for Pos2d {
    type Output = Pos2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: Pos2d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<&Pos2d> for Pos2d {
    type Output = Pos2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: &Pos2d) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<Pos2d> for &Pos2d {
    type Output = Pos2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: Pos2d) -> Self::Output {
        self.compose(&rhs)
    }
}

impl Mul<&Pos2d> for &Pos2d {
    type Output = Pos2d;

    /// Compose two transformations.
    /// `a * b` applies `b` first, then `a`.
    fn mul(self, rhs: &Pos2d) -> Self::Output {
        self.compose(rhs)
    }
}

impl Mul<Vec2> for Pos2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: Vec2) -> Self::Output {
        self.apply(rhs)
    }
}

impl Mul<&Vec2> for Pos2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self.apply(*rhs)
    }
}

impl Mul<Vec2> for &Pos2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: Vec2) -> Self::Output {
        self.apply(rhs)
    }
}

impl Mul<&Vec2> for &Pos2d {
    type Output = Vec2;

    /// Apply the transformation to a point.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self.apply(*rhs)
    }
}

#[cfg(feature = "bevy")]
impl From<Pos2d> for bevy::prelude::Isometry2d {
    fn from(value: Pos2d) -> Self {
        Self {
            rotation: bevy::prelude::Rot2::IDENTITY,
            translation: value.0,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::{Transformation2d, Vec2};
    use crate::transformation::pos2d::Pos2d;

    #[test]
    fn apply_to_origin() {
        let isometry = Pos2d::new(1.0, 2.0);
        assert_eq!(isometry.apply_to_origin(), Vec2::new(1.0, 2.0));
    }

    #[test]
    fn apply() {
        let isometry = Pos2d::new(1.0, 2.0);
        assert_eq!(isometry.apply(Vec2::ONE), Vec2::new(2.0, 3.0));
    }

    #[test]
    fn inverse() {
        let isometry = Pos2d::new(1.0, 2.0);
        assert_eq!(isometry.inverse(), Pos2d::new(-1.0, -2.0));
    }

    #[test]
    fn compose() {
        let a = Pos2d(Vec2::new(1.0, 2.0));
        let b = Pos2d(Vec2::new(3.0, 4.0));
        assert_eq!(a.compose(&b), Pos2d(Vec2::new(4.0, 6.0)));

        let point = Vec2::new(5.0, 7.0);
        assert_eq!(a.apply(b.apply(point)), a.compose(&b).apply(point));
    }

    #[test]
    fn mul_consistency() {
        let a = Pos2d(Vec2::new(1.0, 2.0));
        let b = Pos2d(Vec2::new(3.0, 4.0));
        let point = Vec2::new(5.0, 7.0);

        // * for compose is consistent with apply
        assert_eq!(a.clone() * b.clone(), a.compose(&b));
        assert_eq!(a.clone() * &b, a.compose(&b));
        assert_eq!(&a * b.clone(), a.compose(&b));
        assert_eq!(&a * &b, a.compose(&b));
        assert_eq!((a.clone() * b.clone()) * point, a * (b * point));
    }
}

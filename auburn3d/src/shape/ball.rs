use crate::Vec3;
use crate::property::support::Support;

use super::Shape;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// A ball (sphere) shape defined by a radius.
pub struct Ball {
    pub radius: f32,
}

impl Ball {
    /// Creates a new `Ball` with the given radius.
    pub const fn new(radius: f32) -> Self {
        Self { radius }
    }

    /// Creates a `Collider` with this shape at a given position (isometry).
    pub fn at<'a, I: crate::transformation::Transformation3d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Ball {
    fn volume(&self) -> f32 {
        (4.0 / 3.0) * std::f32::consts::PI * self.radius * self.radius * self.radius
    }
}

impl Support for Ball {
    fn support(&self, dir: Vec3) -> Vec3 {
        dir.normalize_or_zero() * self.radius
    }
}

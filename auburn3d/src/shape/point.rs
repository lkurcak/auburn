use super::Shape;
use crate::Vec3;
use crate::property::support::Support;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
pub struct Point;

impl Point {
    pub fn at<'a, I: crate::transformation::Transformation3d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Point {
    fn volume(&self) -> f32 {
        0.0
    }
}

impl Support for Point {
    fn support(&self, _dir: Vec3) -> Vec3 {
        Vec3::ZERO
    }
}

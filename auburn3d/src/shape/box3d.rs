use crate::Vec3;
use crate::property::support::Support;

use super::Shape;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// A 3D box (cuboid) defined by its half-size (extents).
///
/// The box is centered at the origin of its local frame.
pub struct Box3d {
    pub half_size: Vec3,
}

impl Box3d {
    /// Creates a new `Box3d` from half-extents.
    pub const fn new(half_size: Vec3) -> Self {
        Self { half_size }
    }

    /// Creates a cube `Box3d` with side length `2.0 * half_size`.
    pub const fn cube(half_size: f32) -> Self {
        Self::new(Vec3::new(half_size, half_size, half_size))
    }

    /// Creates a `Collider` with this shape at a given position (isometry).
    pub fn at<'a, I: crate::transformation::Transformation3d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Box3d {
    fn volume(&self) -> f32 {
        self.half_size.x * self.half_size.y * self.half_size.z * 8.0
    }
}

impl Support for Box3d {
    fn support(&self, dir: Vec3) -> Vec3 {
        Vec3::new(
            self.half_size.x.copysign(dir.x),
            self.half_size.y.copysign(dir.y),
            self.half_size.z.copysign(dir.z),
        )
    }
}

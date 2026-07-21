use crate::Vec3;

use crate::{prelude::Support, shape::Shape, transformation::Transformation3d};

#[derive(Debug)]
/// A collision primitive consisting of a shape and a transformation (isometry).
///
/// The `Collider` struct combines a geometric shape (like a `Ball` or `Box3d`) with a
/// transformation (position and orientation) to define a physical object in space.
///
/// # Examples
///
/// ```
/// use auburn3d::prelude::*;
/// use auburn3d::Vec3;
///
/// let ball = Ball::new(1.0);
/// let position = ScaleTranslate3d::from_translation(Vec3::new(10.0, 0.0, 0.0));
/// // You can create a collider directly:
/// let collider = Collider::new(&ball, &position);
/// // Or use the more ergonomic `.at()` method on the shape:
/// let collider_at = ball.at(&position);
/// ```
pub struct Collider<'a, S: Shape, I: Transformation3d> {
    pub shape: &'a S,
    pub isometry: &'a I,
}

impl<S: Shape, I: Transformation3d> Clone for Collider<'_, S, I> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<S: Shape, I: Transformation3d> Copy for Collider<'_, S, I> {}

impl<'a, S: Shape, I: Transformation3d> Collider<'a, S, I> {
    /// Creates a new `Collider` from a shape and a transformation.
    pub const fn new(shape: &'a S, isometry: &'a I) -> Self {
        Self { shape, isometry }
    }

    /// Checks if this collider intersects with another object.
    pub fn collides<T>(&self, other: &T) -> bool
    where
        Self: crate::relation::collides::Collides<T>,
    {
        crate::relation::collides::Collides::collides(self, other)
    }

    /// Checks if this collider penetrates another object and returns the penetration vector.
    ///
    /// Returns `Some(penetration_vector)` if they overlap, or `None` if they don't.
    /// The penetration vector points in the direction to move `self` to resolve the collision.
    pub fn penetrates<T>(&self, other: &T) -> Option<Vec3>
    where
        Self: crate::relation::penetrates::Penetrates<T>,
    {
        crate::relation::penetrates::Penetrates::penetrates(self, other)
    }

    /// Calculates the Time of Impact (TOI) with another object given a relative velocity.
    pub fn toi<T>(&self, other: &T, vel: Vec3) -> Option<f32>
    where
        Self: crate::relation::cast::TimeOfImpact<T>,
    {
        crate::relation::cast::TimeOfImpact::toi(self, other, vel)
    }

    /// Time Traveling Time of Impact (TTTOI).
    pub fn tttoi<T>(&self, other: &T, vel: Vec3) -> Option<f32>
    where
        Self: crate::relation::cast::TimeTravelingTimeOfImpact<T>,
    {
        crate::relation::cast::TimeTravelingTimeOfImpact::tttoi(self, other, vel)
    }

    /// Time of Impact and Exit.
    pub fn toiae<T>(&self, other: &T, vel: Vec3) -> Option<(f32, f32)>
    where
        Self: crate::relation::cast::TimeOfImpactAndExit<T>,
    {
        crate::relation::cast::TimeOfImpactAndExit::toiae(self, other, vel)
    }

    /// Time Traveling Time of Impact and Exit.
    pub fn tttoiae<T>(&self, other: &T, vel: Vec3) -> Option<(f32, f32)>
    where
        Self: crate::relation::cast::TimeTravelingTimeOfImpactAndExit<T>,
    {
        crate::relation::cast::TimeTravelingTimeOfImpactAndExit::tttoiae(self, other, vel)
    }
}

impl<S: Shape, I: Transformation3d> Support for Collider<'_, S, I>
where
    S: Support,
{
    fn support(&self, dir: Vec3) -> Vec3 {
        let local_dir =
            self.isometry.inverse().apply(dir) - self.isometry.inverse().apply_to_origin();
        let local_support = self.shape.support(local_dir);
        self.isometry.apply(local_support)
    }
}

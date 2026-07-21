use crate::Vec2;

use crate::{prelude::Support, shape::Shape, transformation::Transformation2d};

#[derive(Debug)]
/// A collision primitive consisting of a shape and a transformation (isometry).
///
/// The `Collider` struct combines a geometric shape (like a `Ball` or `Box2d`) with a
/// transformation (position and orientation) to define a physical object in space.
///
/// # Examples
///
/// ```
/// use auburn2d::prelude::*;
/// use auburn2d::Vec2;
///
/// let ball = Ball::new(1.0);
/// let position = ScaleTranslate2d::from_translation(Vec2::new(10.0, 0.0));
/// // You can create a collider directly:
/// let collider = Collider::new(&ball, &position);
/// // Or use the more ergonomic `.at()` method on the shape:
/// let collider_at = ball.at(&position);
/// ```
pub struct Collider<'a, S: Shape, I: Transformation2d> {
    pub shape: &'a S,
    pub isometry: &'a I,
}

impl<S: Shape, I: Transformation2d> Clone for Collider<'_, S, I> {
    fn clone(&self) -> Self {
        *self
    }
}

impl<S: Shape, I: Transformation2d> Copy for Collider<'_, S, I> {}

impl<'a, S: Shape, I: Transformation2d> Collider<'a, S, I> {
    /// Creates a new `Collider` from a shape and a transformation.
    pub const fn new(shape: &'a S, isometry: &'a I) -> Self {
        Self { shape, isometry }
    }

    /// Checks if this collider intersects with another object.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::Vec2;
    ///
    /// let ball = Ball::new(1.0);
    /// let t1 = ScaleTranslate2d::from_translation(Vec2::new(0.0, 0.0));
    /// let t2 = ScaleTranslate2d::from_translation(Vec2::new(0.5, 0.0));
    /// let t3 = ScaleTranslate2d::from_translation(Vec2::new(3.0, 0.0));
    ///
    /// assert!(ball.at(&t1).collides(&ball.at(&t2)));
    /// assert!(!ball.at(&t1).collides(&ball.at(&t3)));
    /// ```
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
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::Vec2;
    ///
    /// let ball = Ball::new(1.0);
    /// let t1 = ScaleTranslate2d::from_translation(Vec2::new(0.0, 0.0));
    /// let t2 = ScaleTranslate2d::from_translation(Vec2::new(0.5, 0.0));
    ///
    /// if let Some(pen) = ball.at(&t1).penetrates(&ball.at(&t2)) {
    ///     assert!(pen.length() > 0.0);
    /// } else {
    ///     panic!("Should penetrate");
    /// }
    /// ```
    pub fn penetrates<T>(&self, other: &T) -> Option<Vec2>
    where
        Self: crate::relation::penetrates::Penetrates<T>,
    {
        crate::relation::penetrates::Penetrates::penetrates(self, other)
    }

    /// Calculates the Time of Impact (TOI) with another object given a relative velocity.
    ///
    /// Returns `Some(t)` where `t` is the time (0.0 to infinity) when collision occurs,
    /// or `None` if they don't collide within the movement.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::Vec2;
    ///
    /// let ball = Ball::new(1.0);
    /// let t1 = Vec2::new(0.0, 0.0);
    /// let t2 = Vec2::new(4.0, 0.0);
    ///
    /// // Moving ball at t1 towards ball at t2
    /// let toi = ball.at(&t1).toi(&ball.at(&t2), Vec2::new(5.0, 0.0));
    /// assert!(toi.is_some());
    /// ```
    pub fn toi<T>(&self, other: &T, vel: Vec2) -> Option<f32>
    where
        Self: crate::relation::cast::TimeOfImpact<T>,
    {
        crate::relation::cast::TimeOfImpact::toi(self, other, vel)
    }

    /// Time Traveling Time of Impact (TTTOI).
    /// Similar to `toi`, but handles cases where objects might already be overlapping or starting in a collision state differently.
    pub fn tttoi<T>(&self, other: &T, vel: Vec2) -> Option<f32>
    where
        Self: crate::relation::cast::TimeTravelingTimeOfImpact<T>,
    {
        crate::relation::cast::TimeTravelingTimeOfImpact::tttoi(self, other, vel)
    }

    /// Time of Impact and Exit.
    /// Returns `Some((toi, exit_time))` if collision occurs.
    pub fn toiae<T>(&self, other: &T, vel: Vec2) -> Option<(f32, f32)>
    where
        Self: crate::relation::cast::TimeOfImpactAndExit<T>,
    {
        crate::relation::cast::TimeOfImpactAndExit::toiae(self, other, vel)
    }

    /// Time Traveling Time of Impact and Exit.
    pub fn tttoiae<T>(&self, other: &T, vel: Vec2) -> Option<(f32, f32)>
    where
        Self: crate::relation::cast::TimeTravelingTimeOfImpactAndExit<T>,
    {
        crate::relation::cast::TimeTravelingTimeOfImpactAndExit::tttoiae(self, other, vel)
    }
}

impl<S: Shape, I: Transformation2d> Support for Collider<'_, S, I>
where
    S: Support,
{
    fn support(&self, dir: Vec2) -> Vec2 {
        let local_dir =
            self.isometry.inverse().apply(dir) - self.isometry.inverse().apply_to_origin();
        let local_support = self.shape.support(local_dir);
        self.isometry.apply(local_support)
    }
}

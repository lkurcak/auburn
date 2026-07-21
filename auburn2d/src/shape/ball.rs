use super::Shape;
use crate::Vec2;
use crate::property::support::Support;

#[derive(Debug, Clone, Copy, Default)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// A ball (circle) shape defined by a radius.
///
/// # Examples
///
/// ```
/// use auburn2d::shape::ball::Ball;
///
/// let ball = Ball::new(1.5);
/// assert_eq!(ball.radius, 1.5);
/// ```
pub struct Ball {
    pub radius: f32,
}

impl Ball {
    /// Creates a new `Ball` with the given radius.
    pub const fn new(radius: f32) -> Self {
        Self { radius }
    }

    /// Creates a `Collider` with this shape at a given position (isometry).
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::Vec2;
    ///
    /// let ball = Ball::new(1.0);
    /// let pos = Vec2::new(5.0, 5.0);
    /// let collider = ball.at(&pos);
    ///
    /// assert_eq!(collider.shape.radius, 1.0);
    /// ```
    pub fn at<'a, I: crate::transformation::Transformation2d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Ball {
    fn area(&self) -> f32 {
        std::f32::consts::PI * self.radius * self.radius
    }
}

impl Support for Ball {
    fn support(&self, dir: Vec2) -> Vec2 {
        dir.normalize_or_zero() * self.radius
    }
}

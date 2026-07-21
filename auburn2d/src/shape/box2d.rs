use crate::Vec2;
use crate::property::support::Support;

use super::Shape;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// A 2D box (rectangle) defined by its half-size (extents).
///
/// The box is centered at the origin of its local frame.
/// `half_size` represents the distance from the center to the edges along the X and Y axes.
///
/// # Examples
///
/// ```
/// use auburn2d::prelude::*;
/// use auburn2d::Vec2;
///
/// let box2d = Box2d::new(Vec2::new(1.0, 2.0));
/// assert_eq!(box2d.half_size, Vec2::new(1.0, 2.0));
/// ```
pub struct Box2d {
    pub half_size: Vec2,
}

impl Box2d {
    /// Creates a new `Box2d` from half-extents.
    pub const fn new(half_size: Vec2) -> Self {
        Self { half_size }
    }

    /// Creates a square `Box2d` with side length `2.0 * half_size`.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    ///
    /// let square = Box2d::square(2.0);
    /// assert_eq!(square.half_size.x, 2.0);
    /// assert_eq!(square.half_size.y, 2.0);
    /// ```
    pub const fn square(half_size: f32) -> Self {
        Self::new(Vec2::new(half_size, half_size))
    }

    /// Returns the top-right corner in local coordinates.
    pub fn top_right(&self) -> Vec2 {
        self.half_size
    }

    /// Returns the top-left corner in local coordinates.
    pub fn top_left(&self) -> Vec2 {
        Vec2::new(-self.half_size.x, self.half_size.y)
    }

    /// Returns the bottom-left corner in local coordinates.
    pub fn bottom_left(&self) -> Vec2 {
        -self.half_size
    }

    /// Returns the bottom-right corner in local coordinates.
    pub fn bottom_right(&self) -> Vec2 {
        Vec2::new(self.half_size.x, -self.half_size.y)
    }

    /// Creates a `Collider` with this shape at a given position (isometry).
    pub fn at<'a, I: crate::transformation::Transformation2d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Box2d {
    fn area(&self) -> f32 {
        self.half_size.x * self.half_size.y * 4.0
    }
}

impl Support for Box2d {
    fn support(&self, dir: Vec2) -> Vec2 {
        Vec2::new(
            self.half_size.x.copysign(dir.x),
            self.half_size.y.copysign(dir.y),
        )
    }
}

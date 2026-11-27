use crate::Vec2;
use crate::prelude::*;
use crate::shape::Primitive;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
pub enum Shape2d {
    Point(Point),
    Ball(Ball),
    Box2d(Box2d),
}

impl Default for Shape2d {
    fn default() -> Self {
        Self::Point(Point)
    }
}

impl Shape2d {
    pub fn point() -> Self {
        Self::Point(Point)
    }

    pub fn ball(radius: f32) -> Self {
        Self::Ball(Ball::new(radius))
    }

    pub fn square(half_width: f32) -> Self {
        Self::Box2d(Box2d::new(Vec2::new(half_width, half_width)))
    }

    pub fn box2d(half_width: f32, half_height: f32) -> Self {
        Self::Box2d(Box2d::new(Vec2::new(half_width, half_height)))
    }

    pub fn at<'a, I: crate::transformation::Transformation2d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Shape2d {
    fn area(&self) -> f32 {
        match self {
            Shape2d::Ball(ball) => ball.area(),
            Shape2d::Box2d(box2d) => box2d.area(),
            Shape2d::Point(point) => point.area(),
        }
    }

    /// The variant's shape, so that casts use its exact formulas.
    fn primitive(&self) -> Primitive<'_> {
        match self {
            Shape2d::Ball(ball) => ball.primitive(),
            Shape2d::Box2d(box2d) => box2d.primitive(),
            Shape2d::Point(point) => point.primitive(),
        }
    }
}

impl Support for Shape2d {
    fn support(&self, dir: Vec2) -> Vec2 {
        match self {
            Shape2d::Ball(ball) => ball.support(dir),
            Shape2d::Box2d(box2d) => box2d.support(dir),
            Shape2d::Point(point) => point.support(dir),
        }
    }
}

use crate::{prelude::Collider, transformation::Transformation2d};

pub mod ball;
pub mod box2d;
pub mod point;
pub mod poly2d;
pub mod shape2d;

pub trait Shape {
    fn area(&self) -> f32;

    /// The built-in shape this is, if it is one: casts between some pairs of built-in shapes
    /// have exact formulas, and the rest go through the shapes' support functions.
    ///
    /// The default, [`Primitive::Other`], always works. A shape that wraps a built-in one,
    /// as [`Shape2d`](shape2d::Shape2d) does, can hand it out to get its formulas.
    fn primitive(&self) -> Primitive<'_> {
        Primitive::Other
    }

    fn at<'a, I: Transformation2d>(&'a self, isometry: &'a I) -> Collider<'a, Self, I>
    where
        Self: Sized,
    {
        Collider::new(self, isometry)
    }
}

/// What a shape is, for the casts: see [`Shape::primitive`].
#[derive(Clone, Copy, Debug)]
#[non_exhaustive]
pub enum Primitive<'a> {
    Point,
    Ball(&'a ball::Ball),
    Box2d(&'a box2d::Box2d),
    Poly2d(&'a poly2d::Poly2d),
    /// Any other shape.
    Other,
}

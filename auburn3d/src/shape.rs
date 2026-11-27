use crate::{prelude::Collider, transformation::Transformation3d};

pub mod ball;
pub mod box3d;
pub mod point;

pub trait Shape {
    fn volume(&self) -> f32;

    /// The built-in shape this is, if it is one: casts between some pairs of built-in shapes
    /// have exact formulas, and the rest go through the shapes' support functions.
    ///
    /// The default, [`Primitive::Other`], always works. A shape that wraps a built-in one can
    /// hand it out to get its formulas.
    fn primitive(&self) -> Primitive<'_> {
        Primitive::Other
    }

    fn at<'a, I: Transformation3d>(&'a self, isometry: &'a I) -> Collider<'a, Self, I>
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
    Box3d(&'a box3d::Box3d),
    /// Any other shape.
    Other,
}

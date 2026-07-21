use crate::{prelude::Collider, transformation::Transformation2d};

pub mod ball;
pub mod box2d;
pub mod point;
pub mod poly2d;
pub mod shape2d;

pub trait Shape {
    fn area(&self) -> f32;

    fn at<'a, I: Transformation2d>(&'a self, isometry: &'a I) -> Collider<'a, Self, I>
    where
        Self: Sized,
    {
        Collider::new(self, isometry)
    }
}

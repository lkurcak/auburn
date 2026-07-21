use crate::{prelude::Collider, transformation::Transformation3d};

pub mod ball;
pub mod box3d;
pub mod point;

pub trait Shape {
    fn volume(&self) -> f32;

    fn at<'a, I: Transformation3d>(&'a self, isometry: &'a I) -> Collider<'a, Self, I>
    where
        Self: Sized,
    {
        Collider::new(self, isometry)
    }
}

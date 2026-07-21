use crate::Vec2;
use crate::rotor2d::Rotor2d;

#[cfg(feature = "bevy")]
pub mod bevy_transform;
pub mod pos2d;
pub mod scale_translate2d;
pub mod transform2d;
pub mod vec2;

pub use scale_translate2d::ScaleTranslate2d;
pub use transform2d::Transform2d;

pub trait Transformation2d {
    fn apply_to_origin(&self) -> Vec2;
    fn apply(&self, point: Vec2) -> Vec2;

    fn inverse(&self) -> Self;
    fn compose(&self, other: &Self) -> Self;
}

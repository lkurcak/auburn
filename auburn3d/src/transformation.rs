use crate::Vec3;

#[cfg(feature = "bevy")]
pub mod bevy_transform;
pub mod pos3d;
pub mod scale_translate3d;
pub mod transform3d;
pub mod vec3;

pub use scale_translate3d::ScaleTranslate3d;
pub use transform3d::Transform3d;

pub trait Transformation3d {
    fn apply_to_origin(&self) -> Vec3;
    fn apply(&self, point: Vec3) -> Vec3;

    fn inverse(&self) -> Self;
    fn compose(&self, other: &Self) -> Self;
}

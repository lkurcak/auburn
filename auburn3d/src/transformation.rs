use crate::{Quat, Vec3};

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

    /// Maps a world-space direction to the local-space direction with the same support point:
    /// the transpose of the linear part applied to `dir`, without the translation. Any positive
    /// multiple of it works as well.
    ///
    /// [`Collider`](crate::prelude::Collider)'s support function calls it. The default goes
    /// through [`inverse`](Self::inverse), so it is right only for uniform scale, and far from
    /// the origin it loses short directions to the translation cancelling out. Implement it to
    /// avoid both.
    fn support_direction(&self, dir: Vec3) -> Vec3
    where
        Self: Sized,
    {
        let inverse = self.inverse();
        inverse.apply(dir) - inverse.apply_to_origin()
    }

    /// The linear part, if it is a rotation after a scale along the local axes: the rotation,
    /// and the scale along x, y and z, negative to mirror. Then `apply(p)` is
    /// `rot * (scale * p) + apply_to_origin()`.
    ///
    /// Casts use it to tell when a ball stays a ball, and have exact formulas for some of
    /// those. The default, `None`, is always right: casts then go through the shapes' support
    /// functions.
    fn rotation_and_scale(&self) -> Option<(Quat, Vec3)> {
        None
    }
}

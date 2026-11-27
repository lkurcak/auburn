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

    /// Maps a world-space direction to the local-space direction with the same support point:
    /// the transpose of the linear part applied to `dir`, without the translation. Any positive
    /// multiple of it works as well.
    ///
    /// [`Collider`](crate::prelude::Collider)'s support function calls it. The default goes
    /// through [`inverse`](Self::inverse), so it is right only for uniform scale, and far from
    /// the origin it loses short directions to the translation cancelling out. Implement it to
    /// avoid both.
    fn support_direction(&self, dir: Vec2) -> Vec2
    where
        Self: Sized,
    {
        let inverse = self.inverse();
        inverse.apply(dir) - inverse.apply_to_origin()
    }

    /// The linear part, if it is a rotation after a scale along the local axes: the rotation,
    /// and the scale along x and y, negative to mirror. Then `apply(p)` is
    /// `rot * (scale * p) + apply_to_origin()`.
    ///
    /// Casts use it to tell when a ball stays a ball and a box stays a box, and have exact
    /// formulas for some of those. The default, `None`, is always right: casts then go
    /// through the shapes' support functions.
    fn rotation_and_scale(&self) -> Option<(Rotor2d, Vec2)> {
        None
    }
}

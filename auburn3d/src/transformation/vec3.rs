use crate::Vec3;

use super::Transformation3d;

/// A raw `Vec3` used as a translation-only transformation.
impl Transformation3d for Vec3 {
    fn apply_to_origin(&self) -> Vec3 {
        *self
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point + *self
    }

    fn inverse(&self) -> Self {
        -*self
    }

    fn compose(&self, other: &Self) -> Self {
        self + other
    }
}

use crate::{Quat, Vec3};

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

    fn support_direction(&self, dir: Vec3) -> Vec3 {
        dir
    }

    fn rotation_and_scale(&self) -> Option<(Quat, Vec3)> {
        Some((Quat::IDENTITY, Vec3::ONE))
    }
}

use crate::Vec3;

use super::Transformation3d;

#[derive(Debug, Clone, Copy, PartialEq)]
/// A translation-only transformation.
pub struct Pos3d(pub Vec3);

impl Transformation3d for Pos3d {
    fn apply_to_origin(&self) -> Vec3 {
        self.0
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        point + self.0
    }

    fn inverse(&self) -> Self {
        Pos3d(-self.0)
    }

    fn compose(&self, other: &Self) -> Self {
        Pos3d(self.0 + other.0)
    }
}

impl From<Vec3> for Pos3d {
    fn from(v: Vec3) -> Self {
        Pos3d(v)
    }
}

impl From<Pos3d> for Vec3 {
    fn from(p: Pos3d) -> Self {
        p.0
    }
}

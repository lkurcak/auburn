use crate::Vec3;

pub trait Support {
    /// Returns the furthest point in the direction of the given vector.
    fn support(&self, dir: Vec3) -> Vec3;
}

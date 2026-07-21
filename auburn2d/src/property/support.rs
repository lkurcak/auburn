use crate::Vec2;

pub trait Support {
    /// Returns the furthest point in the direction of the given vector.
    fn support(&self, dir: Vec2) -> Vec2;
}

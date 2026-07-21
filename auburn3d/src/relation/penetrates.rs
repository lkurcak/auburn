use crate::Vec3;

pub trait Penetrates<T> {
    /// Computes the penetration vector of the two objects.
    fn penetrates(&self, other: &T) -> Option<Vec3>;
}

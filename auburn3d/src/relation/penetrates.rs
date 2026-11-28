use crate::Vec3;

pub trait Penetrates<T> {
    /// The shortest move of `self` that parts the shapes, so that they only touch, or `None`
    /// if they don't overlap. Shapes that only touch give a zero vector.
    fn penetrates(&self, other: &T) -> Option<Vec3>;
}

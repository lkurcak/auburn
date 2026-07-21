pub trait Collides<T> {
    /// Returns true if the two objects collide.
    fn collides(&self, other: &T) -> bool;
}

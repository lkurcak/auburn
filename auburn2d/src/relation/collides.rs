pub trait Collides<T> {
    /// Whether the shapes overlap. Touching counts.
    fn collides(&self, other: &T) -> bool;
}

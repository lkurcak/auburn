pub mod epa;
/// Collision detection and penetration algorithms
pub mod gjk;
pub mod toi;

// Re-export for convenience
pub use epa::epa;
pub use gjk::gjk;
pub use toi::shape_cast;

pub mod epa;
/// Collision detection and penetration algorithms
pub mod gjk;
pub mod toi;

// Re-export for convenience
pub use epa::epa;
pub use gjk::gjk;
pub use toi::{
    conservative_advancement_toi, conservative_advancement_toi_unbounded,
    conservative_advancement_toiae, conservative_advancement_toiae_unbounded,
};

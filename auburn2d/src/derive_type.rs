// This module is deprecated and will be removed in the future.
// The Wrapper trait pattern caused recursion issues and made the library too complex.
// Instead, we now require explicit implementations for each transformation type.
//
// If you need to use custom transformation types, you have two options:
// 1. Convert to one of the built-in types (Vec2, Transform2d, Transform2d, or bevy::prelude::Transform)
// 2. Implement all the collision traits explicitly for your type

/// Deprecated: This trait is no longer used by the library's internal implementations.
///
/// Previously, this trait was used to automatically derive implementations for wrapper types,
/// but it caused infinite recursion issues with the trait solver.
#[deprecated(
    since = "0.1.0",
    note = "The Wrapper trait pattern is deprecated. Use explicit conversions to built-in transformation types instead."
)]
pub trait Wrapper {
    type Inner;

    fn to_inner(&self) -> &Self::Inner;
    fn from_inner(v: Self::Inner) -> Self;
}

#![deny(clippy::mod_module_files)]

pub mod algorithm;
mod collider;
pub mod derive_type;
pub mod interactions;
pub mod property;
pub mod relation;
pub mod rotor2d;
pub mod shape;
pub mod transformation;

#[cfg(feature = "bevy")]
pub use bevy::prelude::{Quat, Vec2, Vec3};
#[cfg(not(feature = "bevy"))]
pub use glam::{Quat, Vec2, Vec3};

pub mod prelude {
    pub use crate::collider::Collider;
    // pub use crate::relation::cast::Cast;
    // pub use crate::relation::cast::CastResult;
    pub use crate::Vec2;
    pub use crate::property::support::Support;
    pub use crate::relation::cast::TimeOfImpact;
    pub use crate::relation::cast::TimeOfImpactAndExit;
    pub use crate::relation::cast::TimeTravelingTimeOfImpact;
    pub use crate::relation::cast::TimeTravelingTimeOfImpactAndExit;
    pub use crate::relation::collides::Collides;
    pub use crate::relation::penetrates::Penetrates;
    pub use crate::rotor2d::Rotor2d;
    pub use crate::shape::Shape;
    pub use crate::shape::ball::Ball;
    pub use crate::shape::box2d::Box2d;
    pub use crate::shape::point::Point;
    pub use crate::shape::poly2d::Poly2d;
    pub use crate::shape::shape2d::Shape2d;
    pub use crate::transformation::Transformation2d;
    pub use crate::transformation::scale_translate2d::ScaleTranslate2d;
    pub use crate::transformation::transform2d::Transform2d;
}

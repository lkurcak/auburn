#![deny(clippy::mod_module_files)]

pub mod algorithm;
mod collider;
pub mod interactions;
pub mod property;
pub mod relation;
pub mod shape;
pub mod transformation;

#[cfg(feature = "bevy")]
pub use bevy::prelude::{Quat, Vec3};
#[cfg(not(feature = "bevy"))]
pub use glam::{Quat, Vec3};

pub mod prelude {
    pub use crate::collider::Collider;
    pub use crate::Vec3;
    pub use crate::property::support::Support;
    pub use crate::relation::cast::{
        TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact,
        TimeTravelingTimeOfImpactAndExit,
    };
    pub use crate::relation::collides::Collides;
    pub use crate::relation::penetrates::Penetrates;
    pub use crate::shape::Shape;
    pub use crate::shape::ball::Ball;
    pub use crate::shape::box3d::Box3d;
    pub use crate::shape::point::Point;
    pub use crate::transformation::Transformation3d;
    pub use crate::transformation::scale_translate3d::ScaleTranslate3d;
    pub use crate::transformation::transform3d::Transform3d;
}

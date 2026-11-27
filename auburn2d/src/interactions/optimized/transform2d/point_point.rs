//! A point under a [`Transform2d`] is a point at the transform's position, so these casts use
//! the [`Vec2`] ones.

use crate::Vec2;
use crate::collider::Collider;
use crate::interactions::optimized::vec2::point_point::point_point_overlap;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
    from_span,
};
use crate::shape::point::Point;
use crate::transformation::Transform2d;

macro_rules! point_point_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Point, Transform2d>> for Collider<'_, Point, Transform2d> {
            fn $method(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<$Out> {
                let (a, b) = (self.isometry.pos, other.isometry.pos);
                from_span::$method(point_point_overlap(a, b, vel))
            }
        }
    )*};
}

point_point_impls! {
    TimeOfImpact::toi -> f32;
    TimeOfImpactAndExit::toiae -> (f32, f32);
    TimeTravelingTimeOfImpact::tttoi -> f32;
    TimeTravelingTimeOfImpactAndExit::tttoiae -> (f32, f32);
}

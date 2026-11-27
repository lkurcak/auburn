//! A point under a [`Transform3d`] is a point at the transform's position, so these casts use
//! the [`Vec3`] ones.

use crate::Vec3;
use crate::collider::Collider;
use crate::interactions::optimized::vec3::point_point::point_point_overlap;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
    from_span,
};
use crate::shape::point::Point;
use crate::transformation::Transform3d;

macro_rules! point_point_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Point, Transform3d>> for Collider<'_, Point, Transform3d> {
            fn $method(&self, other: &Collider<'_, Point, Transform3d>, vel: Vec3) -> Option<$Out> {
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

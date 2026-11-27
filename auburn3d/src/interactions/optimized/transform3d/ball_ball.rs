//! Balls under a [`Transform3d`] are cast as world-space balls with the exact [`Vec3`] casts;
//! see `world_ball` in `ball_point`.

use super::ball_point::world_ball;
use crate::Vec3;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::ball::Ball;
use crate::transformation::Transform3d;

macro_rules! ball_ball_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Ball, Transform3d>> for Collider<'_, Ball, Transform3d> {
            fn $method(&self, other: &Collider<'_, Ball, Transform3d>, vel: Vec3) -> Option<$Out> {
                let (a, b) = (world_ball(self), world_ball(other));
                let other = Collider::new(&b, &other.isometry.pos);
                Collider::new(&a, &self.isometry.pos).$method(&other, vel)
            }
        }
    )*};
}

ball_ball_impls! {
    TimeOfImpact::toi -> f32;
    TimeOfImpactAndExit::toiae -> (f32, f32);
    TimeTravelingTimeOfImpact::tttoi -> f32;
    TimeTravelingTimeOfImpactAndExit::tttoiae -> (f32, f32);
}

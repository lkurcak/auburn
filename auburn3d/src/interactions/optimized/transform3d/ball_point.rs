//! A ball under a [`Transform3d`] is a ball of radius `radius * |scale|` at the transform's
//! position: its scale is uniform, and neither rotating nor mirroring a ball changes it. So
//! these casts hand the world-space shapes to the exact [`Vec3`] casts.

use crate::Vec3;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, point::Point};
use crate::transformation::Transform3d;

/// The ball as it is in world space: its radius scaled by `|scale|`.
pub(super) fn world_ball(ball: &Collider<'_, Ball, Transform3d>) -> Ball {
    Ball::new(ball.shape.radius * ball.isometry.scale.abs())
}

macro_rules! point_ball_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Ball, Transform3d>> for Collider<'_, Point, Transform3d> {
            fn $method(&self, other: &Collider<'_, Ball, Transform3d>, vel: Vec3) -> Option<$Out> {
                let ball = world_ball(other);
                let other = Collider::new(&ball, &other.isometry.pos);
                Collider::new(&Point, &self.isometry.pos).$method(&other, vel)
            }
        }
        impl $Trait<Collider<'_, Point, Transform3d>> for Collider<'_, Ball, Transform3d> {
            fn $method(&self, other: &Collider<'_, Point, Transform3d>, vel: Vec3) -> Option<$Out> {
                let ball = world_ball(self);
                let other = Collider::new(&Point, &other.isometry.pos);
                Collider::new(&ball, &self.isometry.pos).$method(&other, vel)
            }
        }
    )*};
}

point_ball_impls! {
    TimeOfImpact::toi -> f32;
    TimeOfImpactAndExit::toiae -> (f32, f32);
    TimeTravelingTimeOfImpact::tttoi -> f32;
    TimeTravelingTimeOfImpactAndExit::tttoiae -> (f32, f32);
}

//! A ball under a [`Transform2d`] is a ball of radius `radius * |scale|` at the transform's
//! position: rotation doesn't change it, and a negative uniform scale in 2D is a half turn. So
//! these casts hand the world-space shapes to the exact [`Vec2`] casts.

use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, point::Point};
use crate::transformation::Transform2d;

/// The ball as it is in world space: its radius scaled by `|scale|`.
pub(super) fn world_ball(ball: &Collider<'_, Ball, Transform2d>) -> Ball {
    Ball::new(ball.shape.radius * ball.isometry.scale.abs())
}

macro_rules! point_ball_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Ball, Transform2d>> for Collider<'_, Point, Transform2d> {
            fn $method(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<$Out> {
                let ball = world_ball(other);
                let other = Collider::new(&ball, &other.isometry.pos);
                Collider::new(&Point, &self.isometry.pos).$method(&other, vel)
            }
        }
        impl $Trait<Collider<'_, Point, Transform2d>> for Collider<'_, Ball, Transform2d> {
            fn $method(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<$Out> {
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

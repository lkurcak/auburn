//! Balls against boxes under a [`Transform3d`] go through the generic cast, with the ball as
//! its centre rounded by its world-space radius; see `world_ball` in `ball_point`.

use super::ball_point::world_ball;
use crate::Vec3;
use crate::algorithm::toi::rounded_shape_cast;
use crate::collider::Collider;
use crate::interactions::optimized::casts_from_span;
use crate::property::support::Support;
use crate::shape::{ball::Ball, box3d::Box3d};
use crate::transformation::Transform3d;

/// When `ball`, moving at `vel`, overlaps `cube`, over all of time.
fn ball_box(
    ball: &Collider<'_, Ball, Transform3d>,
    cube: &Collider<'_, Box3d, Transform3d>,
    vel: Vec3,
) -> Option<(f32, f32)> {
    let centre = ball.isometry.pos;
    let radius = world_ball(ball).radius;
    rounded_shape_cast(|_| centre, |dir| cube.support(dir), radius, vel)
}

casts_from_span!(Ball, Box3d, Transform3d, ball_box);

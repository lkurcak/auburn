//! Balls against boxes have no exact cast yet, so they go through the generic one, with the
//! ball as its centre rounded by its radius.

use crate::Vec3;
use crate::algorithm::toi::rounded_shape_cast;
use crate::collider::Collider;
use crate::interactions::optimized::casts_from_span;
use crate::property::support::Support;
use crate::shape::{ball::Ball, box3d::Box3d};

/// When `ball`, moving at `vel`, overlaps `cube`, over all of time.
fn ball_box(
    ball: &Collider<'_, Ball, Vec3>,
    cube: &Collider<'_, Box3d, Vec3>,
    vel: Vec3,
) -> Option<(f32, f32)> {
    let centre = *ball.isometry;
    let radius = ball.shape.radius;
    rounded_shape_cast(|_| centre, |dir| cube.support(dir), radius, vel)
}

casts_from_span!(Ball, Box3d, Vec3, ball_box);

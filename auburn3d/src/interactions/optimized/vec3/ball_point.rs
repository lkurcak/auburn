use crate::Vec3;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
    direction_and_speed, from_span, is_at_rest,
};
use crate::shape::{ball::Ball, point::Point};

/// When a point at `p`, relative to the centre of a ball of `radius`, is in the ball while it
/// moves at `vel`, over all of time (touching counts): from when to when, or `None` if never.
/// At rest, a point in the ball is in it forever.
fn point_ball_overlap(p: Vec3, radius: f32, vel: Vec3) -> Option<(f32, f32)> {
    if is_at_rest(vel) {
        return (p.length_squared() <= radius * radius)
            .then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    // Along the path, in units of distance: the point passes closest to the centre at
    // `-along`, `miss` away from it, and is in the ball for a half chord either side.
    let (dir, speed) = direction_and_speed(vel);
    let along = p.dot(dir);
    let miss = p - dir * along;
    let half_chord_squared = radius * radius - miss.length_squared();
    if half_chord_squared < 0.0 {
        return None;
    }
    let half_chord = half_chord_squared.sqrt();
    Some(((-along - half_chord) / speed, (-along + half_chord) / speed))
}

macro_rules! point_ball_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Ball, Vec3>> for Collider<'_, Point, Vec3> {
            fn $method(&self, other: &Collider<'_, Ball, Vec3>, vel: Vec3) -> Option<$Out> {
                let p = *self.isometry - *other.isometry;
                from_span::$method(point_ball_overlap(p, other.shape.radius, vel))
            }
        }
        impl $Trait<Collider<'_, Point, Vec3>> for Collider<'_, Ball, Vec3> {
            fn $method(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<$Out> {
                // The ball's centre moves through a ball of the same radius around the point.
                let p = *self.isometry - *other.isometry;
                from_span::$method(point_ball_overlap(p, self.shape.radius, vel))
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

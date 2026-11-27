use crate::Vec3;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
    direction_and_speed, from_span, is_at_rest,
};
use crate::shape::point::Point;

/// When a point at `a` coincides with one at `b` while it moves at `vel`, over all of time: the
/// moment it passes through, `None` if it misses by more than rounding, or forever if it is at
/// rest on the other point.
pub(crate) fn point_point_overlap(a: Vec3, b: Vec3, vel: Vec3) -> Option<(f32, f32)> {
    let p = a - b;
    if is_at_rest(vel) {
        return (p == Vec3::ZERO).then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    let (dir, speed) = direction_and_speed(vel);
    let along = p.dot(dir);
    let miss = p - dir * along;
    // Positions are only good to a unit in their last place, so a path straight through can
    // seem to miss by a few of them.
    (miss.length() <= 4.0 * f32::EPSILON * (a.length() + b.length())).then(|| {
        let t = -along / speed;
        (t, t)
    })
}

macro_rules! point_point_impls {
    ($($Trait:ident::$method:ident -> $Out:ty;)*) => {$(
        impl $Trait<Collider<'_, Point, Vec3>> for Collider<'_, Point, Vec3> {
            fn $method(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<$Out> {
                from_span::$method(point_point_overlap(*self.isometry, *other.isometry, vel))
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

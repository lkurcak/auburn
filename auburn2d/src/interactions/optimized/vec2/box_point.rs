use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{box2d::Box2d, point::Point};
use crate::transformation::Transformation2d;

/// When something moving at `vel` along one axis is within `lo..=hi` of where it starts: from
/// when to when (unbounded in both directions if it doesn't move), or `None` if never.
///
/// A velocity too small for its reciprocal to be finite (zero of either sign, or subnormal)
/// counts as not moving: the slab method would otherwise multiply 0 by infinity where the
/// motion starts exactly on `lo` or `hi`.
pub(super) fn slab(lo: f32, hi: f32, vel: f32) -> Option<(f32, f32)> {
    let inv_vel = 1.0 / vel;
    if !inv_vel.is_finite() {
        return (lo <= 0.0 && 0.0 <= hi).then_some((f32::NEG_INFINITY, f32::INFINITY));
    }
    let (a, b) = (lo * inv_vel, hi * inv_vel);
    if a <= b { Some((a, b)) } else { Some((b, a)) }
}

/// When the point overlaps the box, over all of time (touching counts): from when to when, or
/// `None` if never.
fn overlap(
    point: &Collider<'_, Point, Vec2>,
    b: &Collider<'_, Box2d, Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let p = b.isometry.apply_to_origin() - point.isometry.apply_to_origin();
    let half = b.shape.half_size;
    let (x_enter, x_exit) = slab(p.x - half.x, p.x + half.x, vel.x)?;
    let (y_enter, y_exit) = slab(p.y - half.y, p.y + half.y, vel.y)?;
    let enter = x_enter.max(y_enter);
    let exit = x_exit.min(y_exit);
    (enter <= exit).then_some((enter, exit))
}

impl TimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn toi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (enter, exit) = overlap(self, other, vel)?;
        (exit >= 0.0).then_some(enter.max(0.0))
    }
}
impl TimeOfImpact<Collider<'_, Point, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.toi(&b, vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn toiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let (enter, exit) = overlap(self, other, vel)?;
        (exit >= 0.0).then_some((enter.max(0.0), exit))
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.toiae(&b, vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        overlap(self, other, vel).map(|(enter, _)| enter)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoi(&b, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        overlap(self, other, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoiae(&b, vel)
    }
}

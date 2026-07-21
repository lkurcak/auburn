use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::point::Point;

impl TimeOfImpact<Collider<'_, Point, Vec2>> for Collider<'_, Point, Vec2> {
    fn toi(&self, _: &Collider<'_, Point, Vec2>, _: Vec2) -> Option<f32> {
        None
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Vec2>> for Collider<'_, Point, Vec2> {
    fn tttoi(&self, _: &Collider<'_, Point, Vec2>, _: Vec2) -> Option<f32> {
        None
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Point, Vec2> {
    fn toiae(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let toi = self.toi(other, vel);
        toi.map(|toi| (toi, toi))
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Point, Vec2> {
    fn tttoiae(&self, _: &Collider<'_, Point, Vec2>, _: Vec2) -> Option<(f32, f32)> {
        None
    }
}

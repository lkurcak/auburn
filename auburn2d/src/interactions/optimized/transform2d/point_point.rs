use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::point::Point;
use crate::transformation::Transform2d;

impl TimeOfImpact<Collider<'_, Point, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toi(&self, _: &Collider<'_, Point, Transform2d>, _: Vec2) -> Option<f32> {
        None
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoi(&self, _: &Collider<'_, Point, Transform2d>, _: Vec2) -> Option<f32> {
        None
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let toi = self.toi(other, vel);
        toi.map(|toi| (toi, toi))
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoiae(&self, _: &Collider<'_, Point, Transform2d>, _: Vec2) -> Option<(f32, f32)> {
        None
    }
}

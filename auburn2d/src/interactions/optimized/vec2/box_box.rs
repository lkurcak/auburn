use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{box2d::Box2d, point::Point};

impl TimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let big_box = Box2d::new(self.shape.half_size + other.shape.half_size);
        let b = Collider::new(&big_box, other.isometry);
        a.toi(&b, vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let big_box = Box2d::new(self.shape.half_size + other.shape.half_size);
        let b = Collider::new(&big_box, other.isometry);
        a.toiae(&b, vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let big_box = Box2d::new(self.shape.half_size + other.shape.half_size);
        let b = Collider::new(&big_box, other.isometry);
        a.tttoi(&b, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let big_box = Box2d::new(self.shape.half_size + other.shape.half_size);
        let b = Collider::new(&big_box, other.isometry);
        a.tttoiae(&b, vel)
    }
}

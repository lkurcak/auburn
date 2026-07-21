use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, point::Point};

impl TimeOfImpact<Collider<'_, Ball, Vec2>> for Collider<'_, Ball, Vec2> {
    fn toi(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(self.shape.radius + other.shape.radius);
        let b = Collider::new(&big_ball, other.isometry);
        a.toi(&b, vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Ball, Vec2>> for Collider<'_, Ball, Vec2> {
    fn toiae(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(self.shape.radius + other.shape.radius);
        let b = Collider::new(&big_ball, other.isometry);
        a.toiae(&b, vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Vec2>> for Collider<'_, Ball, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(self.shape.radius + other.shape.radius);
        let b = Collider::new(&big_ball, other.isometry);
        a.tttoi(&b, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Vec2>> for Collider<'_, Ball, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(self.shape.radius + other.shape.radius);
        let b = Collider::new(&big_ball, other.isometry);
        a.tttoiae(&b, vel)
    }
}

use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, point::Point};
use crate::transformation::Transform2d;

impl TimeOfImpact<Collider<'_, Ball, Transform2d>> for Collider<'_, Ball, Transform2d> {
    fn toi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(
            self.shape.radius * self.isometry.scale + other.shape.radius * other.isometry.scale,
        );
        let b = Collider::new(&big_ball, other.isometry);
        a.toi(&b, vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Ball, Transform2d>> for Collider<'_, Ball, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(
            self.shape.radius * self.isometry.scale + other.shape.radius * other.isometry.scale,
        );
        let b = Collider::new(&big_ball, other.isometry);
        a.toiae(&b, vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Transform2d>>
    for Collider<'_, Ball, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(
            self.shape.radius * self.isometry.scale + other.shape.radius * other.isometry.scale,
        );
        let b = Collider::new(&big_ball, other.isometry);
        a.tttoi(&b, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Transform2d>>
    for Collider<'_, Ball, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let big_ball = Ball::new(
            self.shape.radius * self.isometry.scale + other.shape.radius * other.isometry.scale,
        );
        let b = Collider::new(&big_ball, other.isometry);
        a.tttoiae(&b, vel)
    }
}

use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, box2d::Box2d};
use crate::transformation::{Transform2d, Transformation2d};

impl TimeOfImpact<Collider<'_, Box2d, Transform2d>> for Collider<'_, Ball, Transform2d> {
    fn toi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_ball_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_radius = self.shape.radius * self.isometry.scale / other.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        ball_collider.toi(&box2d, local_vel)
    }
}
impl TimeOfImpact<Collider<'_, Ball, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_ball_center = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let local_radius = other.shape.radius * other.isometry.scale / self.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        ball_collider.toi(&box2d, -local_vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>> for Collider<'_, Ball, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_ball_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_radius = self.shape.radius * self.isometry.scale / other.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        ball_collider.toiae(&box2d, local_vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Ball, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_ball_center = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let local_radius = other.shape.radius * other.isometry.scale / self.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        ball_collider.toiae(&box2d, -local_vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Ball, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_ball_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_radius = self.shape.radius * self.isometry.scale / other.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        ball_collider.tttoi(&box2d, local_vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_ball_center = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let local_radius = other.shape.radius * other.isometry.scale / self.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        ball_collider.tttoi(&box2d, -local_vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Ball, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_ball_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_radius = self.shape.radius * self.isometry.scale / other.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        ball_collider.tttoiae(&box2d, local_vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_ball_center = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let local_radius = other.shape.radius * other.isometry.scale / self.isometry.scale;
        let ball = Ball::new(local_radius);
        let ball_collider = Collider::new(&ball, &local_ball_center);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        ball_collider.tttoiae(&box2d, -local_vel)
    }
}

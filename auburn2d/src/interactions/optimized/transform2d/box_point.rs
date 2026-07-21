use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{box2d::Box2d, point::Point};
use crate::transformation::{Transform2d, Transformation2d};

impl TimeOfImpact<Collider<'_, Box2d, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_point = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        point.toi(&box2d, local_vel)
    }
}
impl TimeOfImpact<Collider<'_, Point, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_point = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        point.toi(&box2d, -local_vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_point = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        point.toiae(&box2d, local_vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_point = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        point.toiae(&box2d, -local_vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_point = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        point.tttoi(&box2d, local_vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_point = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        point.tttoi(&box2d, -local_vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_point = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(other.shape, &Vec2::ZERO);
        point.tttoiae(&box2d, local_vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_point = self
            .isometry
            .inverse()
            .apply(other.isometry.apply_to_origin());
        let local_vel = self.isometry.rot.inverse() * vel / self.isometry.scale;
        let point = Collider::new(&Point, &local_point);
        let box2d = Collider::new(self.shape, &Vec2::ZERO);
        point.tttoiae(&box2d, -local_vel)
    }
}

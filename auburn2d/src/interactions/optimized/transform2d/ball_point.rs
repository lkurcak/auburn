use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, point::Point};
use crate::transformation::{Transform2d, Transformation2d};

impl TimeOfImpact<Collider<'_, Ball, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
        let d = self.isometry.apply_to_origin() - other.isometry.apply_to_origin();

        let other_radius = other.shape.radius * other.isometry.scale;

        let dd = d.dot(d);
        if dd < other_radius * other_radius {
            return Some(0.0);
        }

        let a = vel.dot(vel);
        let b = 2.0 * d.dot(vel);
        let c = dd - other_radius * other_radius;

        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            None
        } else {
            let sqrt_discriminant = discriminant.sqrt();
            let t1 = (-b - sqrt_discriminant) / (2.0 * a);
            let t2 = (-b + sqrt_discriminant) / (2.0 * a);

            if t1 > 0.0 {
                Some(t1)
            } else if t2 > 0.0 {
                Some(0.0)
            } else {
                None
            }
        }
    }
}
impl TimeOfImpact<Collider<'_, Point, Transform2d>> for Collider<'_, Ball, Transform2d> {
    fn toi(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.toi(&b, vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Ball, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let d = self.isometry.apply_to_origin() - other.isometry.apply_to_origin();

        let other_radius = other.shape.radius * other.isometry.scale;

        let dd = d.dot(d);
        if dd < other_radius * other_radius {
            return Some((0.0, 0.0));
        }

        let a = vel.dot(vel);
        let b = 2.0 * d.dot(vel);
        let c = dd - other_radius * other_radius;

        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            None
        } else {
            let sqrt_discriminant = discriminant.sqrt();
            let t1 = (-b - sqrt_discriminant) / (2.0 * a);
            let t2 = (-b + sqrt_discriminant) / (2.0 * a);

            if t2 < 0.0 {
                None
            } else {
                let toi = t1.max(0.0);
                let toe = t2;
                Some((toi, toe))
            }
        }
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Transform2d>> for Collider<'_, Ball, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.toiae(&b, vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
        let d = self.isometry.apply_to_origin() - other.isometry.apply_to_origin();

        let other_radius = other.shape.radius * other.isometry.scale;

        let dd = d.dot(d);

        let a = vel.dot(vel);
        let b = 2.0 * d.dot(vel);
        let c = dd - other_radius * other_radius;

        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            None
        } else {
            let sqrt_discriminant = discriminant.sqrt();
            let t1 = (-b - sqrt_discriminant) / (2.0 * a);
            let t2 = (-b + sqrt_discriminant) / (2.0 * a);

            Some(t1.min(t2))
        }
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Transform2d>>
    for Collider<'_, Ball, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoi(&b, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let d = self.isometry.apply_to_origin() - other.isometry.apply_to_origin();

        let other_radius = other.shape.radius * other.isometry.scale;

        let dd = d.dot(d);

        let a = vel.dot(vel);
        let b = 2.0 * d.dot(vel);
        let c = dd - other_radius * other_radius;

        let discriminant = b * b - 4.0 * a * c;

        if discriminant < 0.0 {
            None
        } else {
            let sqrt_discriminant = discriminant.sqrt();
            let t1 = (-b - sqrt_discriminant) / (2.0 * a);
            let t2 = (-b + sqrt_discriminant) / (2.0 * a);

            Some((t1, t2))
        }
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Transform2d>>
    for Collider<'_, Ball, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoiae(&b, vel)
    }
}

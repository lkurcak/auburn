use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{box2d::Box2d, point::Point};
use crate::transformation::Transformation2d;

impl TimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn toi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let a = self.isometry.apply_to_origin();
        let b = other.isometry.apply_to_origin();
        let p = b - a;
        let inv_vel = 1.0 / vel;

        let mut tmin = (p - other.shape.half_size) * inv_vel;
        let mut tmax = (p + other.shape.half_size) * inv_vel;

        if vel.x.is_sign_negative() {
            std::mem::swap(&mut tmin.x, &mut tmax.x);
        }
        if vel.y.is_sign_negative() {
            std::mem::swap(&mut tmin.y, &mut tmax.y);
        }

        let t0 = tmin.x.max(tmin.y);
        let t1 = tmax.x.min(tmax.y);

        if t0 > t1 || t1 < 0.0 {
            return None;
        }

        if t0 < 0.0 { Some(0.0) } else { Some(t0) }
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
        let a = self.isometry.apply_to_origin();
        let b = other.isometry.apply_to_origin();
        let p = b - a;
        let inv_vel = 1.0 / vel;

        let mut tmin = (p - other.shape.half_size) * inv_vel;
        let mut tmax = (p + other.shape.half_size) * inv_vel;

        if vel.x.is_sign_negative() {
            std::mem::swap(&mut tmin.x, &mut tmax.x);
        }
        if vel.y.is_sign_negative() {
            std::mem::swap(&mut tmin.y, &mut tmax.y);
        }

        let t0 = tmin.x.max(tmin.y);
        let t1 = tmax.x.min(tmax.y);

        if t0 > t1 || t1 < 0.0 {
            return None;
        }

        let toi = t0.max(0.0);
        let toe = t1;

        Some((toi, toe))
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
        let a = self.isometry.apply_to_origin();
        let b = other.isometry.apply_to_origin();
        let p = b - a;
        let inv_vel = 1.0 / vel;

        let mut tmin = (p - other.shape.half_size) * inv_vel;
        let mut tmax = (p + other.shape.half_size) * inv_vel;

        if vel.x.is_sign_negative() {
            std::mem::swap(&mut tmin.x, &mut tmax.x);
        }
        if vel.y.is_sign_negative() {
            std::mem::swap(&mut tmin.y, &mut tmax.y);
        }

        let t0 = tmin.x.max(tmin.y);
        let t1 = tmax.x.min(tmax.y);

        if t0 > t1 {
            return None;
        }

        Some(t0)
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
        let a = self.isometry.apply_to_origin();
        let b = other.isometry.apply_to_origin();
        let p = b - a;
        let inv_vel = 1.0 / vel;

        let mut tmin = (p - other.shape.half_size) * inv_vel;
        let mut tmax = (p + other.shape.half_size) * inv_vel;

        if vel.x.is_sign_negative() {
            std::mem::swap(&mut tmin.x, &mut tmax.x);
        }
        if vel.y.is_sign_negative() {
            std::mem::swap(&mut tmin.y, &mut tmax.y);
        }

        let t0 = tmin.x.max(tmin.y);
        let t1 = tmax.x.min(tmax.y);

        if t0 > t1 {
            return None;
        }

        Some((t0, t1))
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(&Point, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoiae(&b, vel)
    }
}

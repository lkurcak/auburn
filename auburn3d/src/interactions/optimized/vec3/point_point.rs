use crate::Vec3;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::point::Point;

impl TimeOfImpact<Collider<'_, Point, Vec3>> for Collider<'_, Point, Vec3> {
    fn toi(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<f32> {
        // |self.isometry + vel*t - other.isometry|^2 = 0
        let a = self.isometry - other.isometry;
        let b = vel;

        let a_coeff = b.dot(b);
        let b_coeff = 2.0 * a.dot(b);
        let c_coeff = a.dot(a);

        if a_coeff.abs() < 1e-10 {
            if c_coeff < 1e-10 {
                return Some(0.0);
            }
            return None;
        }

        let discriminant = b_coeff * b_coeff - 4.0 * a_coeff * c_coeff;
        if discriminant < 0.0 {
            return None;
        }

        let sqrt_disc = discriminant.sqrt();
        let t1 = (-b_coeff - sqrt_disc) / (2.0 * a_coeff);
        let t2 = (-b_coeff + sqrt_disc) / (2.0 * a_coeff);

        let t_min = t1.min(t2);
        if t_min >= 0.0 {
            Some(t_min)
        } else if t1.max(t2) >= 0.0 {
            Some(0.0)
        } else {
            None
        }
    }
}

impl TimeOfImpactAndExit<Collider<'_, Point, Vec3>> for Collider<'_, Point, Vec3> {
    fn toiae(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        // Point-point: exit time is infinity (or the same as entry if already colliding)
        self.toi(other, vel).map(|t| (t, f32::INFINITY))
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Point, Vec3>> for Collider<'_, Point, Vec3> {
    fn tttoi(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<f32> {
        self.toi(other, vel)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Vec3>> for Collider<'_, Point, Vec3> {
    fn tttoiae(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        self.toiae(other, vel)
    }
}

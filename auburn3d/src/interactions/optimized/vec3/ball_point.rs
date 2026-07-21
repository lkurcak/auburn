use crate::Vec3;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, point::Point};

impl TimeOfImpact<Collider<'_, Point, Vec3>> for Collider<'_, Ball, Vec3> {
    fn toi(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<f32> {
        // Solve: |self.isometry + vel*t - other.isometry|^2 = self.radius^2
        let a = self.isometry - other.isometry;
        let b = vel;
        let c = self.shape.radius * self.shape.radius;

        // |a + b*t|^2 = c
        // (b.b)*t^2 + 2(a.b)*t + (a.a - c) = 0
        let a_coeff = b.dot(b);
        let b_coeff = 2.0 * a.dot(b);
        let c_coeff = a.dot(a) - c;

        solve_quadratic_toi(a_coeff, b_coeff, c_coeff)
    }
}

impl TimeOfImpactAndExit<Collider<'_, Point, Vec3>> for Collider<'_, Ball, Vec3> {
    fn toiae(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        let a = self.isometry - other.isometry;
        let b = vel;
        let c = self.shape.radius * self.shape.radius;

        let a_coeff = b.dot(b);
        let b_coeff = 2.0 * a.dot(b);
        let c_coeff = a.dot(a) - c;

        solve_quadratic_toiae(a_coeff, b_coeff, c_coeff)
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Point, Vec3>> for Collider<'_, Ball, Vec3> {
    fn tttoi(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<f32> {
        self.toi(other, vel)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Vec3>> for Collider<'_, Ball, Vec3> {
    fn tttoiae(&self, other: &Collider<'_, Point, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        self.toiae(other, vel)
    }
}

// Reverse: Point vs Ball
impl TimeOfImpact<Collider<'_, Ball, Vec3>> for Collider<'_, Point, Vec3> {
    fn toi(&self, other: &Collider<'_, Ball, Vec3>, vel: Vec3) -> Option<f32> {
        other.toi(self, -vel)
    }
}

impl TimeOfImpactAndExit<Collider<'_, Ball, Vec3>> for Collider<'_, Point, Vec3> {
    fn toiae(&self, other: &Collider<'_, Ball, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        other.toiae(self, -vel).map(|(exit, entry)| (entry, exit))
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Vec3>> for Collider<'_, Point, Vec3> {
    fn tttoi(&self, other: &Collider<'_, Ball, Vec3>, vel: Vec3) -> Option<f32> {
        other.tttoi(self, -vel)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Vec3>> for Collider<'_, Point, Vec3> {
    fn tttoiae(&self, other: &Collider<'_, Ball, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        other.tttoiae(self, -vel).map(|(exit, entry)| (entry, exit))
    }
}

fn solve_quadratic_toi(a: f32, b: f32, c: f32) -> Option<f32> {
    if a.abs() < 1e-10 {
        // Linear equation
        if b.abs() < 1e-10 {
            return None;
        }
        let t = -c / b;
        if t >= 0.0 {
            return Some(t);
        }
        return None;
    }

    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }

    let sqrt_disc = discriminant.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);

    let t_min = t1.min(t2);
    let t_max = t1.max(t2);

    if t_min >= 0.0 {
        Some(t_min)
    } else if t_max >= 0.0 {
        Some(0.0)
    } else {
        None
    }
}

fn solve_quadratic_toiae(a: f32, b: f32, c: f32) -> Option<(f32, f32)> {
    if a.abs() < 1e-10 {
        if b.abs() < 1e-10 {
            return None;
        }
        let t = -c / b;
        if t >= 0.0 {
            return Some((t, f32::INFINITY));
        }
        return None;
    }

    let discriminant = b * b - 4.0 * a * c;
    if discriminant < 0.0 {
        return None;
    }

    let sqrt_disc = discriminant.sqrt();
    let t1 = (-b - sqrt_disc) / (2.0 * a);
    let t2 = (-b + sqrt_disc) / (2.0 * a);

    let t_min = t1.min(t2);
    let t_max = t1.max(t2);

    if t_max < 0.0 {
        return None;
    }

    Some((t_min.max(0.0), t_max))
}

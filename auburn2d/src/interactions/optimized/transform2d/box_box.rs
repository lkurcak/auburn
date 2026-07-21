use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{box2d::Box2d, point::Point};
use crate::transformation::{Transform2d, Transformation2d};

/// Compute the time interval during which |c0 + v * t| <= r.
///
/// Returns `(t_min, t_max)` where the projection intervals overlap.
/// If there is no overlap at any time, returns `(inf, -inf)`.
fn interval_overlap(c0: f32, v: f32, r: f32) -> (f32, f32) {
    let abs_c0 = c0.abs();
    if v.abs() < 1e-10 {
        if abs_c0 <= r {
            (f32::NEG_INFINITY, f32::INFINITY)
        } else {
            (f32::INFINITY, f32::NEG_INFINITY)
        }
    } else {
        let t1 = (-r - c0) / v;
        let t2 = (r - c0) / v;
        (t1.min(t2), t1.max(t2))
    }
}

/// Transform self into other's local space and compute the exact
/// AABB vs OBB overlap interval using SAT.
fn box_box_overlap_interval(
    self_isometry: &Transform2d,
    self_shape: &Box2d,
    other_isometry: &Transform2d,
    other_shape: &Box2d,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let local_center = other_isometry
        .inverse()
        .apply(self_isometry.apply_to_origin());
    let local_vel = other_isometry.rot.inverse() * vel / other_isometry.scale;
    let local_rot = other_isometry.rot.inverse() * self_isometry.rot;
    let local_scale = self_isometry.scale / other_isometry.scale;
    let local_half_size = self_shape.half_size * local_scale;
    let b_half = other_shape.half_size;

    let (cos, sin) = local_rot.to_cos_sin();

    let mut t_entry = f32::NEG_INFINITY;
    let mut t_exit = f32::INFINITY;

    // AABB B's X axis
    {
        let r = b_half.x + local_half_size.x * cos.abs() + local_half_size.y * sin.abs();
        let (min, max) = interval_overlap(local_center.x, local_vel.x, r);
        t_entry = t_entry.max(min);
        t_exit = t_exit.min(max);
        if t_entry > t_exit {
            return None;
        }
    }

    // AABB B's Y axis
    {
        let r = b_half.y + local_half_size.x * sin.abs() + local_half_size.y * cos.abs();
        let (min, max) = interval_overlap(local_center.y, local_vel.y, r);
        t_entry = t_entry.max(min);
        t_exit = t_exit.min(max);
        if t_entry > t_exit {
            return None;
        }
    }

    // OBB A's X axis
    {
        let c0 = local_center.x * cos + local_center.y * sin;
        let v = local_vel.x * cos + local_vel.y * sin;
        let r = b_half.x * cos.abs() + b_half.y * sin.abs() + local_half_size.x;
        let (min, max) = interval_overlap(c0, v, r);
        t_entry = t_entry.max(min);
        t_exit = t_exit.min(max);
        if t_entry > t_exit {
            return None;
        }
    }

    // OBB A's Y axis
    {
        let c0 = -local_center.x * sin + local_center.y * cos;
        let v = -local_vel.x * sin + local_vel.y * cos;
        let r = b_half.x * sin.abs() + b_half.y * cos.abs() + local_half_size.y;
        let (min, max) = interval_overlap(c0, v, r);
        t_entry = t_entry.max(min);
        t_exit = t_exit.min(max);
        if t_entry > t_exit {
            return None;
        }
    }

    Some((t_entry, t_exit))
}

impl TimeOfImpact<Collider<'_, Box2d, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_rot = other.isometry.rot.inverse() * self.isometry.rot;
        let local_scale = self.isometry.scale / other.isometry.scale;
        let local_half_size = self.shape.half_size * local_scale;
        let b_half = other.shape.half_size;

        let (cos, sin) = local_rot.to_cos_sin();

        // Fast path: same rotation (0° or 180° relative) → both are AABBs in B's local space
        if sin.abs() < 1e-4 {
            let effective_half = Vec2::new(
                local_half_size.x * cos.abs() + local_half_size.y * sin.abs(),
                local_half_size.x * sin.abs() + local_half_size.y * cos.abs(),
            );
            let big_box = Box2d::new(effective_half + b_half);
            let point = Collider::new(&Point, &local_center);
            let box_collider = Collider::new(&big_box, &Vec2::ZERO);
            return point.toi(&box_collider, local_vel);
        }

        let (t_entry, t_exit) =
            box_box_overlap_interval(self.isometry, self.shape, other.isometry, other.shape, vel)?;

        if t_exit < 0.0 {
            return None;
        }

        Some(t_entry.max(0.0))
    }
}

impl TimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_rot = other.isometry.rot.inverse() * self.isometry.rot;
        let local_scale = self.isometry.scale / other.isometry.scale;
        let local_half_size = self.shape.half_size * local_scale;
        let b_half = other.shape.half_size;

        let (cos, sin) = local_rot.to_cos_sin();

        // Fast path: same rotation
        if sin.abs() < 1e-4 {
            let effective_half = Vec2::new(
                local_half_size.x * cos.abs() + local_half_size.y * sin.abs(),
                local_half_size.x * sin.abs() + local_half_size.y * cos.abs(),
            );
            let big_box = Box2d::new(effective_half + b_half);
            let point = Collider::new(&Point, &local_center);
            let box_collider = Collider::new(&big_box, &Vec2::ZERO);
            return point.toiae(&box_collider, local_vel);
        }

        let (t_entry, t_exit) =
            box_box_overlap_interval(self.isometry, self.shape, other.isometry, other.shape, vel)?;

        if t_exit < 0.0 {
            return None;
        }

        Some((t_entry.max(0.0), t_exit))
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let local_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_rot = other.isometry.rot.inverse() * self.isometry.rot;
        let local_scale = self.isometry.scale / other.isometry.scale;
        let local_half_size = self.shape.half_size * local_scale;
        let b_half = other.shape.half_size;

        let (cos, sin) = local_rot.to_cos_sin();

        // Fast path: same rotation
        if sin.abs() < 1e-4 {
            let effective_half = Vec2::new(
                local_half_size.x * cos.abs() + local_half_size.y * sin.abs(),
                local_half_size.x * sin.abs() + local_half_size.y * cos.abs(),
            );
            let big_box = Box2d::new(effective_half + b_half);
            let point = Collider::new(&Point, &local_center);
            let box_collider = Collider::new(&big_box, &Vec2::ZERO);
            return point.tttoi(&box_collider, local_vel);
        }

        let (t_entry, t_exit) =
            box_box_overlap_interval(self.isometry, self.shape, other.isometry, other.shape, vel)?;

        if t_entry > t_exit {
            return None;
        }

        Some(t_entry)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let local_center = other
            .isometry
            .inverse()
            .apply(self.isometry.apply_to_origin());
        let local_vel = other.isometry.rot.inverse() * vel / other.isometry.scale;
        let local_rot = other.isometry.rot.inverse() * self.isometry.rot;
        let local_scale = self.isometry.scale / other.isometry.scale;
        let local_half_size = self.shape.half_size * local_scale;
        let b_half = other.shape.half_size;

        let (cos, sin) = local_rot.to_cos_sin();

        // Fast path: same rotation
        if sin.abs() < 1e-4 {
            let effective_half = Vec2::new(
                local_half_size.x * cos.abs() + local_half_size.y * sin.abs(),
                local_half_size.x * sin.abs() + local_half_size.y * cos.abs(),
            );
            let big_box = Box2d::new(effective_half + b_half);
            let point = Collider::new(&Point, &local_center);
            let box_collider = Collider::new(&big_box, &Vec2::ZERO);
            return point.tttoiae(&box_collider, local_vel);
        }

        let (t_entry, t_exit) =
            box_box_overlap_interval(self.isometry, self.shape, other.isometry, other.shape, vel)?;

        if t_entry > t_exit {
            return None;
        }

        Some((t_entry, t_exit))
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::prelude::*;

    #[test]
    fn test_box_box_transform2d_toi_stationary() {
        let shape_a = Box2d::square(1.0);
        let pos_a = Transform2d::from_translation(Vec2::new(-3.0, 0.0));
        let a = shape_a.at(&pos_a);

        let shape_b = Box2d::square(1.0);
        let pos_b = Transform2d::from_translation(Vec2::new(0.0, 0.0));
        let b = shape_b.at(&pos_b);

        // A moves right at vel 1, should hit at t = 1.0 (surface at -2 hits surface at -1)
        assert_eq!(a.toi(&b, Vec2::new(1.0, 0.0)), Some(1.0));
        // No Y movement, should miss
        assert_eq!(a.toi(&b, Vec2::new(0.0, 1.0)), None);
    }

    #[test]
    fn test_box_box_transform2d_toi_scale() {
        let shape_a = Box2d::square(1.0);
        let pos_a = Transform2d::from_translation(Vec2::new(-4.0, 0.0));
        let a = shape_a.at(&pos_a);

        let shape_b = Box2d::square(1.0);
        let pos_b = Transform2d::from_translation(Vec2::new(0.0, 0.0)).with_scale(2.0);
        let b = shape_b.at(&pos_b);

        // A moves right at vel 1
        // A's surface at x=-3, B's surface at x=-2
        // Distance = 1.0, toi = 1.0
        assert_eq!(a.toi(&b, Vec2::new(1.0, 0.0)), Some(1.0));
    }

    #[test]
    fn test_box_box_transform2d_toi_rotation_45() {
        let shape_a = Box2d::square(1.0);
        let pos_a = Transform2d::from_translation(Vec2::new(-3.0, 0.0));
        let a = shape_a.at(&pos_a);

        let shape_b = Box2d::square(1.0);
        let pos_b = Transform2d::from_translation(Vec2::new(0.0, 0.0))
            .with_angle(std::f32::consts::FRAC_PI_4);
        let b = shape_b.at(&pos_b);

        // A moves right at vel 1
        let toi = a.toi(&b, Vec2::new(1.0, 0.0));
        println!("45° rotation toi: {:?}", toi);
        assert!(toi.is_some());
        let t = toi.unwrap();
        // The 45° square's leftmost point is at (-sqrt(2), 0) ≈ (-1.414, 0)
        // Box A's right face is at x=-2 (center -3 + half_size 1)
        // Distance from -2 to -1.414 is 0.586, so toi = 2 - sqrt(2) ≈ 0.5858
        assert!((t - 0.5857864).abs() < 1e-4, "Expected ~0.586, got {}", t);
    }

    #[test]
    fn test_box_box_transform2d_toi_already_overlapping() {
        let shape_a = Box2d::square(1.0);
        let pos_a = Transform2d::from_translation(Vec2::new(-1.0, 0.0));
        let a = shape_a.at(&pos_a);

        let shape_b = Box2d::square(1.0);
        let pos_b = Transform2d::from_translation(Vec2::new(0.0, 0.0));
        let b = shape_b.at(&pos_b);

        // Already overlapping
        assert_eq!(a.toi(&b, Vec2::new(1.0, 0.0)), Some(0.0));
    }

    #[test]
    fn test_box_box_transform2d_toiae() {
        let shape_a = Box2d::square(1.0);
        let pos_a = Transform2d::from_translation(Vec2::new(-5.0, 0.0));
        let a = shape_a.at(&pos_a);

        let shape_b = Box2d::square(1.0);
        let pos_b = Transform2d::from_translation(Vec2::new(0.0, 0.0));
        let b = shape_b.at(&pos_b);

        // A moves right at vel 1
        // Entry: A's right face (-4) hits B's left face (-1) → distance 3 → t = 3
        // Exit: A's left face (-6) hits B's right face (1) → distance 7 → t = 7
        let toiae = a.toiae(&b, Vec2::new(1.0, 0.0));
        assert_eq!(toiae, Some((3.0, 7.0)));
    }

    #[test]
    fn test_box_box_transform2d_tttoi() {
        let shape_a = Box2d::square(1.0);
        let pos_a = Transform2d::from_translation(Vec2::new(-1.0, 0.0));
        let a = shape_a.at(&pos_a);

        let shape_b = Box2d::square(1.0);
        let pos_b = Transform2d::from_translation(Vec2::new(0.0, 0.0));
        let b = shape_b.at(&pos_b);

        let tttoi = a.tttoi(&b, Vec2::new(1.0, 0.0));
        println!("tttoi already overlapping: {:?}", tttoi);
        assert!(tttoi.is_some());
        assert!(tttoi.unwrap() <= 0.0);
    }
}

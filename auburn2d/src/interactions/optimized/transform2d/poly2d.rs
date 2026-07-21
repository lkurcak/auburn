use crate::Vec2;
use crate::algorithm::toi::{
    conservative_advancement_toi, conservative_advancement_toi_unbounded,
    conservative_advancement_toiae, conservative_advancement_toiae_unbounded,
};
use crate::collider::Collider;
use crate::interactions::optimized::poly2d_utils::{
    point_poly_toi, point_poly_toiae, point_poly_tttoi, point_poly_tttoiae, swept_sat_ttoiae,
};
use crate::prelude::{Ball, Box2d, Point, Poly2d};
use crate::property::support::Support;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::transformation::{Transform2d, Transformation2d};

// ============================================================================
// Poly2d vs Point
// ============================================================================

impl TimeOfImpact<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| other.isometry.apply(p))
            .collect();
        point_poly_toi(origin, vel, &vertices)
    }
}
impl TimeOfImpact<Collider<'_, Point, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<f32> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| self.isometry.apply(p))
            .collect();
        point_poly_toi(origin, -vel, &vertices)
    }
}

impl TimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Point, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| other.isometry.apply(p))
            .collect();
        point_poly_toiae(origin, vel, &vertices)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| self.isometry.apply(p))
            .collect();
        point_poly_toiae(origin, -vel, &vertices)
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| other.isometry.apply(p))
            .collect();
        point_poly_tttoi(origin, vel, &vertices)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Transform2d>>
    for Collider<'_, Poly2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<f32> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| self.isometry.apply(p))
            .collect();
        point_poly_tttoi(origin, -vel, &vertices)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>>
    for Collider<'_, Point, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| other.isometry.apply(p))
            .collect();
        point_poly_tttoiae(origin, vel, &vertices)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Transform2d>>
    for Collider<'_, Poly2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Point, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| self.isometry.apply(p))
            .collect();
        point_poly_tttoiae(origin, -vel, &vertices)
    }
}

// ============================================================================
// Poly2d vs Ball  (kept as conservative advancement)
// ============================================================================

macro_rules! poly2d_ball_impls {
    () => {
        impl TimeOfImpact<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Ball, Transform2d> {
            fn toi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi(|dir| self.support(dir), |dir| other.support(dir), vel)
            }
        }
        impl TimeOfImpact<Collider<'_, Ball, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
            fn toi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi(|dir| self.support(dir), |dir| other.support(dir), vel)
            }
        }
        impl TimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>>
            for Collider<'_, Ball, Transform2d>
        {
            fn toiae(
                &self,
                other: &Collider<'_, Poly2d, Transform2d>,
                vel: Vec2,
            ) -> Option<(f32, f32)> {
                conservative_advancement_toiae(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeOfImpactAndExit<Collider<'_, Ball, Transform2d>>
            for Collider<'_, Poly2d, Transform2d>
        {
            fn toiae(
                &self,
                other: &Collider<'_, Ball, Transform2d>,
                vel: Vec2,
            ) -> Option<(f32, f32)> {
                conservative_advancement_toiae(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Transform2d>>
            for Collider<'_, Ball, Transform2d>
        {
            fn tttoi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Transform2d>>
            for Collider<'_, Poly2d, Transform2d>
        {
            fn tttoi(&self, other: &Collider<'_, Ball, Transform2d>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>>
            for Collider<'_, Ball, Transform2d>
        {
            fn tttoiae(
                &self,
                other: &Collider<'_, Poly2d, Transform2d>,
                vel: Vec2,
            ) -> Option<(f32, f32)> {
                conservative_advancement_toiae_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Transform2d>>
            for Collider<'_, Poly2d, Transform2d>
        {
            fn tttoiae(
                &self,
                other: &Collider<'_, Ball, Transform2d>,
                vel: Vec2,
            ) -> Option<(f32, f32)> {
                conservative_advancement_toiae_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
    };
}

poly2d_ball_impls!();

// ============================================================================
// Poly2d vs Box2d
// ============================================================================

fn get_axes(vertices: &[Vec2]) -> Vec<Vec2> {
    let n = vertices.len();
    (0..n)
        .map(|i| {
            let j = (i + 1) % n;
            let edge = vertices[j] - vertices[i];
            Vec2::new(-edge.y, edge.x)
        })
        .collect()
}

fn project(vertices: &[Vec2], axis: Vec2) -> (f32, f32) {
    let mut min = f32::INFINITY;
    let mut max = f32::NEG_INFINITY;
    for &v in vertices {
        let proj = v.dot(axis);
        if proj < min {
            min = proj;
        }
        if proj > max {
            max = proj;
        }
    }
    (min, max)
}

fn poly_box_sat(
    box_collider: &Collider<'_, Box2d, Transform2d>,
    poly_collider: &Collider<'_, Poly2d, Transform2d>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let box_vertices = [
        box_collider.isometry.apply(box_collider.shape.top_right()),
        box_collider.isometry.apply(box_collider.shape.top_left()),
        box_collider
            .isometry
            .apply(box_collider.shape.bottom_left()),
        box_collider
            .isometry
            .apply(box_collider.shape.bottom_right()),
    ];
    let poly_vertices: Vec<Vec2> = poly_collider
        .shape
        .points
        .iter()
        .map(|&p| poly_collider.isometry.apply(p))
        .collect();

    let mut axes = get_axes(&box_vertices);
    axes.extend(get_axes(&poly_vertices));

    let get_proj_box = |axis: Vec2| project(&box_vertices, axis);
    let get_proj_poly = |axis: Vec2| project(&poly_vertices, axis);

    swept_sat_ttoiae(get_proj_box, get_proj_poly, &axes, vel)
}

impl TimeOfImpact<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let (t_enter, t_exit) = poly_box_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some(t_enter.max(0.0))
    }
}
impl TimeOfImpact<Collider<'_, Box2d, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let (t_enter, t_exit) = poly_box_sat(other, self, -vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some(t_enter.max(0.0))
    }
}

impl TimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Box2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let (t_enter, t_exit) = poly_box_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some((t_enter.max(0.0), t_exit))
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let (t_enter, t_exit) = poly_box_sat(other, self, -vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some((t_enter.max(0.0), t_exit))
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let (t_enter, _) = poly_box_sat(self, other, vel)?;
        Some(t_enter)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Poly2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let (t_enter, _) = poly_box_sat(other, self, -vel)?;
        Some(t_enter)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>>
    for Collider<'_, Box2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        poly_box_sat(self, other, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Transform2d>>
    for Collider<'_, Poly2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Box2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        poly_box_sat(other, self, -vel)
    }
}

// ============================================================================
// Poly2d vs Poly2d
// ============================================================================

fn poly_poly_sat(
    poly_a: &Collider<'_, Poly2d, Transform2d>,
    poly_b: &Collider<'_, Poly2d, Transform2d>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let verts_a: Vec<Vec2> = poly_a
        .shape
        .points
        .iter()
        .map(|&p| poly_a.isometry.apply(p))
        .collect();
    let verts_b: Vec<Vec2> = poly_b
        .shape
        .points
        .iter()
        .map(|&p| poly_b.isometry.apply(p))
        .collect();

    let mut axes = get_axes(&verts_a);
    axes.extend(get_axes(&verts_b));

    let get_proj_a = |axis: Vec2| project(&verts_a, axis);
    let get_proj_b = |axis: Vec2| project(&verts_b, axis);

    swept_sat_ttoiae(get_proj_a, get_proj_b, &axes, vel)
}

impl TimeOfImpact<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
    fn toi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let (t_enter, t_exit) = poly_poly_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some(t_enter.max(0.0))
    }
}
impl TimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>> for Collider<'_, Poly2d, Transform2d> {
    fn toiae(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        let (t_enter, t_exit) = poly_poly_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some((t_enter.max(0.0), t_exit))
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Transform2d>>
    for Collider<'_, Poly2d, Transform2d>
{
    fn tttoi(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<f32> {
        let (t_enter, _) = poly_poly_sat(self, other, vel)?;
        Some(t_enter)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Transform2d>>
    for Collider<'_, Poly2d, Transform2d>
{
    fn tttoiae(&self, other: &Collider<'_, Poly2d, Transform2d>, vel: Vec2) -> Option<(f32, f32)> {
        poly_poly_sat(self, other, vel)
    }
}

#[cfg(test)]
mod tests {
    use crate::prelude::*;
    use std::sync::Arc;

    fn unit_square() -> Poly2d {
        Poly2d::new(Arc::new(vec![
            Vec2::new(-1.0, -1.0),
            Vec2::new(1.0, -1.0),
            Vec2::new(1.0, 1.0),
            Vec2::new(-1.0, 1.0),
        ]))
    }

    #[test]
    fn test_poly_point_transform2d_toi() {
        let square = unit_square();
        let poly_tr = Transform2d::from_translation(Vec2::new(3.0, 0.0));
        let poly = square.at(&poly_tr);
        let point = Point;
        let point_tr = Transform2d::IDENTITY;
        let point_collider = point.at(&point_tr);

        let toi = point_collider.toi(&poly, Vec2::new(1.0, 0.0));
        assert!(
            (toi.unwrap() - 2.0).abs() < 1e-4,
            "Expected toi=2.0, got {:?}",
            toi
        );
    }

    #[test]
    fn test_poly_point_transform2d_toi_rotated() {
        let square = unit_square();
        let poly_tr = Transform2d::from_translation(Vec2::new(3.0, 0.0))
            .with_angle(std::f32::consts::FRAC_PI_4);
        let poly = square.at(&poly_tr);
        let point = Point;
        let point_tr = Transform2d::IDENTITY;
        let point_collider = point.at(&point_tr);

        let toi = point_collider.toi(&poly, Vec2::new(1.0, 0.0));
        assert!(toi.is_some());
        let t = toi.unwrap();
        assert!((t - 1.5858).abs() < 1e-3, "Expected toi≈1.586, got {}", t);
    }

    #[test]
    fn test_poly_box_transform2d_toi() {
        let square = unit_square();
        let poly_tr = Transform2d::from_translation(Vec2::new(4.0, 0.0));
        let poly = square.at(&poly_tr);
        let bx = Box2d::square(1.0);
        let bx_tr = Transform2d::IDENTITY;
        let bx_collider = bx.at(&bx_tr);

        let toi = bx_collider.toi(&poly, Vec2::new(1.0, 0.0));
        assert!(
            (toi.unwrap() - 2.0).abs() < 1e-4,
            "Expected toi=2.0, got {:?}",
            toi
        );
    }

    #[test]
    fn test_poly_box_transform2d_toi_rotated() {
        let square = unit_square();
        let poly_tr = Transform2d::from_translation(Vec2::new(4.0, 0.0))
            .with_angle(std::f32::consts::FRAC_PI_4);
        let poly = square.at(&poly_tr);
        let bx = Box2d::square(1.0);
        let bx_tr = Transform2d::IDENTITY;
        let bx_collider = bx.at(&bx_tr);

        let toi = bx_collider.toi(&poly, Vec2::new(1.0, 0.0));
        assert!(toi.is_some());
        let expected = 4.0 - std::f32::consts::SQRT_2 - 1.0;
        let t = toi.unwrap();
        assert!(
            (t - expected).abs() < 1e-3,
            "Expected toi≈{expected}, got {t}"
        );
    }

    #[test]
    fn test_poly_poly_transform2d_toi() {
        let square = unit_square();
        let poly_a_tr = Transform2d::from_translation(Vec2::new(-4.0, 0.0));
        let poly_a = square.at(&poly_a_tr);
        let poly_b_tr = Transform2d::from_translation(Vec2::new(0.0, 0.0));
        let poly_b = square.at(&poly_b_tr);

        let toi = poly_a.toi(&poly_b, Vec2::new(1.0, 0.0));
        assert!(
            (toi.unwrap() - 2.0).abs() < 1e-4,
            "Expected toi=2.0, got {:?}",
            toi
        );
    }

    #[test]
    fn test_poly_poly_transform2d_toi_rotated() {
        let square = unit_square();
        let poly_a_tr = Transform2d::from_translation(Vec2::new(-4.0, 0.0));
        let poly_a = square.at(&poly_a_tr);
        let poly_b_tr = Transform2d::from_translation(Vec2::new(0.0, 0.0))
            .with_angle(std::f32::consts::FRAC_PI_4);
        let poly_b = square.at(&poly_b_tr);

        let toi = poly_a.toi(&poly_b, Vec2::new(1.0, 0.0));
        assert!(toi.is_some());
        let expected = 3.0 - std::f32::consts::SQRT_2;
        let t = toi.unwrap();
        assert!(
            (t - expected).abs() < 1e-3,
            "Expected toi≈{expected}, got {t}"
        );
    }
}

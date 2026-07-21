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
use crate::transformation::Transformation2d;

// ============================================================================
// Poly2d vs Point
// ============================================================================

impl TimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn toi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| *other.isometry + p)
            .collect();
        point_poly_toi(origin, vel, &vertices)
    }
}
impl TimeOfImpact<Collider<'_, Point, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<f32> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| *self.isometry + p)
            .collect();
        point_poly_toi(origin, -vel, &vertices)
    }
}

impl TimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn toiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| *other.isometry + p)
            .collect();
        point_poly_toiae(origin, vel, &vertices)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| *self.isometry + p)
            .collect();
        point_poly_toiae(origin, -vel, &vertices)
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| *other.isometry + p)
            .collect();
        point_poly_tttoi(origin, vel, &vertices)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Point, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<f32> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| *self.isometry + p)
            .collect();
        point_poly_tttoi(origin, -vel, &vertices)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Point, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = self.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = other
            .shape
            .points
            .iter()
            .map(|&p| *other.isometry + p)
            .collect();
        point_poly_tttoiae(origin, vel, &vertices)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Point, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let origin = other.isometry.apply_to_origin();
        let vertices: Vec<Vec2> = self
            .shape
            .points
            .iter()
            .map(|&p| *self.isometry + p)
            .collect();
        point_poly_tttoiae(origin, -vel, &vertices)
    }
}

// ============================================================================
// Poly2d vs Ball  (kept as conservative advancement – exact ball-poly is complex)
// ============================================================================

macro_rules! poly2d_ball_impls {
    () => {
        impl TimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Ball, Vec2> {
            fn toi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi(|dir| self.support(dir), |dir| other.support(dir), vel)
            }
        }
        impl TimeOfImpact<Collider<'_, Ball, Vec2>> for Collider<'_, Poly2d, Vec2> {
            fn toi(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi(|dir| self.support(dir), |dir| other.support(dir), vel)
            }
        }
        impl TimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Ball, Vec2> {
            fn toiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
                conservative_advancement_toiae(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeOfImpactAndExit<Collider<'_, Ball, Vec2>> for Collider<'_, Poly2d, Vec2> {
            fn toiae(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
                conservative_advancement_toiae(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Ball, Vec2> {
            fn tttoi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Vec2>> for Collider<'_, Poly2d, Vec2> {
            fn tttoi(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<f32> {
                conservative_advancement_toi_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>>
            for Collider<'_, Ball, Vec2>
        {
            fn tttoiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
                conservative_advancement_toiae_unbounded(
                    |dir| self.support(dir),
                    |dir| other.support(dir),
                    vel,
                )
            }
        }
        impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Vec2>>
            for Collider<'_, Poly2d, Vec2>
        {
            fn tttoiae(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
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

fn poly_box_sat(
    box_collider: &Collider<'_, Box2d, Vec2>,
    poly_collider: &Collider<'_, Poly2d, Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let poly_axes: Vec<Vec2> = poly_collider
        .shape
        .points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let j = (i + 1) % poly_collider.shape.points.len();
            let q = poly_collider.shape.points[j];
            Vec2::new(-(q.y - p.y), q.x - p.x)
        })
        .collect();

    let mut axes = poly_axes;
    axes.push(Vec2::X);
    axes.push(Vec2::Y);

    let get_proj_poly = |axis: Vec2| {
        let offset = poly_collider.isometry.dot(axis);
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for &p in poly_collider.shape.points.iter() {
            let proj = p.dot(axis) + offset;
            if proj < min {
                min = proj;
            }
            if proj > max {
                max = proj;
            }
        }
        (min, max)
    };

    let get_proj_box = |axis: Vec2| {
        let offset = box_collider.isometry.dot(axis);
        let hx = box_collider.shape.half_size.x;
        let hy = box_collider.shape.half_size.y;
        let min = -hx * axis.x.abs() - hy * axis.y.abs() + offset;
        let max = hx * axis.x.abs() + hy * axis.y.abs() + offset;
        (min, max)
    };

    swept_sat_ttoiae(get_proj_box, get_proj_poly, &axes, vel)
}

impl TimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (t_enter, t_exit) = poly_box_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some(t_enter.max(0.0))
    }
}
impl TimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (t_enter, t_exit) = poly_box_sat(other, self, -vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some(t_enter.max(0.0))
    }
}

impl TimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let (t_enter, t_exit) = poly_box_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some((t_enter.max(0.0), t_exit))
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let (t_enter, t_exit) = poly_box_sat(other, self, -vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some((t_enter.max(0.0), t_exit))
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (t_enter, _) = poly_box_sat(self, other, vel)?;
        Some(t_enter)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (t_enter, _) = poly_box_sat(other, self, -vel)?;
        Some(t_enter)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        poly_box_sat(self, other, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        poly_box_sat(other, self, -vel)
    }
}

// ============================================================================
// Poly2d vs Poly2d
// ============================================================================

fn poly_poly_sat(
    poly_a: &Collider<'_, Poly2d, Vec2>,
    poly_b: &Collider<'_, Poly2d, Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let mut axes: Vec<Vec2> = poly_a
        .shape
        .points
        .iter()
        .enumerate()
        .map(|(i, p)| {
            let j = (i + 1) % poly_a.shape.points.len();
            let q = poly_a.shape.points[j];
            Vec2::new(-(q.y - p.y), q.x - p.x)
        })
        .collect();

    axes.extend(poly_b.shape.points.iter().enumerate().map(|(i, p)| {
        let j = (i + 1) % poly_b.shape.points.len();
        let q = poly_b.shape.points[j];
        Vec2::new(-(q.y - p.y), q.x - p.x)
    }));

    let get_proj_a = |axis: Vec2| {
        let offset = poly_a.isometry.dot(axis);
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for &p in poly_a.shape.points.iter() {
            let proj = p.dot(axis) + offset;
            if proj < min {
                min = proj;
            }
            if proj > max {
                max = proj;
            }
        }
        (min, max)
    };

    let get_proj_b = |axis: Vec2| {
        let offset = poly_b.isometry.dot(axis);
        let mut min = f32::INFINITY;
        let mut max = f32::NEG_INFINITY;
        for &p in poly_b.shape.points.iter() {
            let proj = p.dot(axis) + offset;
            if proj < min {
                min = proj;
            }
            if proj > max {
                max = proj;
            }
        }
        (min, max)
    };

    swept_sat_ttoiae(get_proj_a, get_proj_b, &axes, vel)
}

impl TimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (t_enter, t_exit) = poly_poly_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some(t_enter.max(0.0))
    }
}
impl TimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let (t_enter, t_exit) = poly_poly_sat(self, other, vel)?;
        if t_exit < 0.0 {
            return None;
        }
        Some((t_enter.max(0.0), t_exit))
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Poly2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<f32> {
        let (t_enter, _) = poly_poly_sat(self, other, vel)?;
        Some(t_enter)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Poly2d, Vec2>> for Collider<'_, Poly2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Poly2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
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
    fn test_poly_point_vec2_toi() {
        let square = unit_square();
        let poly_pos = Vec2::new(3.0, 0.0);
        let poly = square.at(&poly_pos);
        let point = Point;
        let point_pos = Vec2::ZERO;
        let point_collider = point.at(&point_pos);

        let toi = point_collider.toi(&poly, Vec2::new(1.0, 0.0));
        assert!(
            (toi.unwrap() - 2.0).abs() < 1e-4,
            "Expected toi=2.0, got {:?}",
            toi
        );
    }

    #[test]
    fn test_poly_point_vec2_toi_inside() {
        let square = unit_square();
        let poly_pos = Vec2::ZERO;
        let poly = square.at(&poly_pos);
        let point = Point;
        let point_pos = Vec2::ZERO;
        let point_collider = point.at(&point_pos);

        assert_eq!(point_collider.toi(&poly, Vec2::new(1.0, 0.0)), Some(0.0));
    }

    #[test]
    fn test_poly_point_vec2_toiae() {
        let square = unit_square();
        let poly_pos = Vec2::new(4.0, 0.0);
        let poly = square.at(&poly_pos);
        let point = Point;
        let point_pos = Vec2::ZERO;
        let point_collider = point.at(&point_pos);

        let toiae = point_collider.toiae(&poly, Vec2::new(1.0, 0.0));
        assert_eq!(toiae, Some((3.0, 5.0)));
    }

    #[test]
    fn test_poly_box_vec2_toi() {
        let square = unit_square();
        let poly_pos = Vec2::new(4.0, 0.0);
        let poly = square.at(&poly_pos);
        let bx = Box2d::square(1.0);
        let bx_pos = Vec2::ZERO;
        let bx_collider = bx.at(&bx_pos);

        let toi = bx_collider.toi(&poly, Vec2::new(1.0, 0.0));
        assert!(
            (toi.unwrap() - 2.0).abs() < 1e-4,
            "Expected toi=2.0, got {:?}",
            toi
        );
    }

    #[test]
    fn test_poly_box_vec2_toi_overlapping() {
        let square = unit_square();
        let poly_pos = Vec2::new(1.0, 0.0);
        let poly = square.at(&poly_pos);
        let bx = Box2d::square(1.0);
        let bx_pos = Vec2::ZERO;
        let bx_collider = bx.at(&bx_pos);

        assert_eq!(bx_collider.toi(&poly, Vec2::new(1.0, 0.0)), Some(0.0));
    }

    #[test]
    fn test_poly_poly_vec2_toi() {
        let square = unit_square();
        let poly_a_pos = Vec2::new(-4.0, 0.0);
        let poly_a = square.at(&poly_a_pos);
        let poly_b_pos = Vec2::new(0.0, 0.0);
        let poly_b = square.at(&poly_b_pos);

        let toi = poly_a.toi(&poly_b, Vec2::new(1.0, 0.0));
        assert!(
            (toi.unwrap() - 2.0).abs() < 1e-4,
            "Expected toi=2.0, got {:?}",
            toi
        );
    }

    #[test]
    fn test_poly_poly_vec2_toi_overlapping() {
        let square = unit_square();
        let poly_a_pos = Vec2::new(-1.0, 0.0);
        let poly_a = square.at(&poly_a_pos);
        let poly_b_pos = Vec2::new(0.0, 0.0);
        let poly_b = square.at(&poly_b_pos);

        assert_eq!(poly_a.toi(&poly_b, Vec2::new(1.0, 0.0)), Some(0.0));
    }
}

use crate::Vec2;
use crate::algorithm::toi::{
    conservative_advancement_toi, conservative_advancement_toi_unbounded,
    conservative_advancement_toiae, conservative_advancement_toiae_unbounded,
};
use crate::collider::Collider;
use crate::interactions::optimized::poly2d_utils::{
    casts_from_span, edge_normals, point_polygon, project, swept_sat,
};
use crate::prelude::{Ball, Box2d, Point, Poly2d};
use crate::property::support::Support;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::transformation::{Transform2d, Transformation2d};

// Each overlap is worked out in world space.

/// The polygon's vertices in world space.
fn world_vertices(poly: &Collider<'_, Poly2d, Transform2d>) -> Vec<Vec2> {
    let vertices = poly.shape.vertices().iter();
    vertices.map(|&p| poly.isometry.apply(p)).collect()
}

// ============================================================================
// Poly2d vs Point
// ============================================================================

/// When `point`, moving at `vel`, is in `poly`, over all of time.
fn point_poly(
    point: &Collider<'_, Point, Transform2d>,
    poly: &Collider<'_, Poly2d, Transform2d>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let p = point.isometry.apply_to_origin();
    point_polygon(p, &world_vertices(poly), vel)
}

casts_from_span!(Point, Poly2d, Transform2d, point_poly);

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

/// When `bx`, moving at `vel`, overlaps `poly`, over all of time.
fn box_poly(
    bx: &Collider<'_, Box2d, Transform2d>,
    poly: &Collider<'_, Poly2d, Transform2d>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let corners = [
        bx.shape.top_right(),
        bx.shape.top_left(),
        bx.shape.bottom_left(),
        bx.shape.bottom_right(),
    ];
    let box_vertices = corners.map(|p| bx.isometry.apply(p));
    let poly_vertices = world_vertices(poly);
    let axes = edge_normals(&box_vertices).chain(edge_normals(&poly_vertices));
    let span_box = |axis| project(&box_vertices, axis);
    let span_poly = |axis| project(&poly_vertices, axis);
    swept_sat(span_box, span_poly, axes, vel)
}

casts_from_span!(Box2d, Poly2d, Transform2d, box_poly);

// ============================================================================
// Poly2d vs Poly2d
// ============================================================================

/// When `a`, moving at `vel`, overlaps `b`, over all of time.
fn poly_poly(
    a: &Collider<'_, Poly2d, Transform2d>,
    b: &Collider<'_, Poly2d, Transform2d>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let (va, vb) = (world_vertices(a), world_vertices(b));
    let axes = edge_normals(&va).chain(edge_normals(&vb));
    swept_sat(
        |axis| project(&va, axis),
        |axis| project(&vb, axis),
        axes,
        vel,
    )
}

casts_from_span!(Poly2d, Transform2d, poly_poly);

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

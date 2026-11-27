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
use crate::transformation::Transformation2d;

// Each overlap is worked out in the polygon's frame, where its vertices are as given.

// ============================================================================
// Poly2d vs Point
// ============================================================================

/// When `point`, moving at `vel`, is in `poly`, over all of time.
fn point_poly(
    point: &Collider<'_, Point, Vec2>,
    poly: &Collider<'_, Poly2d, Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let p = point.isometry.apply_to_origin() - poly.isometry.apply_to_origin();
    point_polygon(p, poly.shape.vertices(), vel)
}

casts_from_span!(Point, Poly2d, Vec2, point_poly);

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

/// When `bx`, moving at `vel`, overlaps `poly`, over all of time.
fn box_poly(
    bx: &Collider<'_, Box2d, Vec2>,
    poly: &Collider<'_, Poly2d, Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let centre = bx.isometry.apply_to_origin() - poly.isometry.apply_to_origin();
    let half = bx.shape.half_size;
    let span_box = |axis: Vec2| {
        let (c, r) = (centre.dot(axis), half.dot(axis.abs()));
        (c - r, c + r)
    };
    let vertices = poly.shape.vertices();
    let span_poly = |axis| project(vertices, axis);
    swept_sat(span_box, span_poly, edge_normals(vertices), vel)
}

casts_from_span!(Box2d, Poly2d, Vec2, box_poly);

// ============================================================================
// Poly2d vs Poly2d
// ============================================================================

/// When `a`, moving at `vel`, overlaps `b`, over all of time.
fn poly_poly(
    a: &Collider<'_, Poly2d, Vec2>,
    b: &Collider<'_, Poly2d, Vec2>,
    vel: Vec2,
) -> Option<(f32, f32)> {
    let offset = a.isometry.apply_to_origin() - b.isometry.apply_to_origin();
    let (va, vb) = (a.shape.vertices(), b.shape.vertices());
    let span_a = |axis: Vec2| {
        let (lo, hi) = project(va, axis);
        let d = offset.dot(axis);
        (lo + d, hi + d)
    };
    let axes = edge_normals(va).chain(edge_normals(vb));
    swept_sat(span_a, |axis| project(vb, axis), axes, vel)
}

casts_from_span!(Poly2d, Vec2, poly_poly);

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

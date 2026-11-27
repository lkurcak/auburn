//! Casts between the built-in shapes, worked out by hand.

use auburn2d::prelude::*;
use std::sync::Arc;

fn unit_square() -> Poly2d {
    Poly2d::new(Arc::new(vec![
        Vec2::new(-1.0, -1.0),
        Vec2::new(1.0, -1.0),
        Vec2::new(1.0, 1.0),
        Vec2::new(-1.0, 1.0),
    ]))
}

/// Polygons placed by a `Vec2`.
mod polygons_by_vec2 {
    use super::*;

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

/// Polygons placed by a `Transform2d`.
mod polygons_by_transform2d {
    use super::*;

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

/// Boxes placed by a `Transform2d`.
mod boxes_by_transform2d {
    use super::*;

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
        assert!(tttoi.is_some());
        assert!(tttoi.unwrap() <= 0.0);
    }
}

/// Pairs that had no casts before every pair went through one generic cast.
mod every_pair {
    use super::*;
    use auburn2d::transformation::pos2d::Pos2d;

    fn near(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
        got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-4 && (b - want.1).abs() < 1e-4)
    }

    #[test]
    fn pos2d() {
        let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
        let (a, b) = (Pos2d::new(-5.0, 0.0), Pos2d::new(0.0, 0.5));
        let got = ball.at(&a).toiae(&square.at(&b), Vec2::new(2.0, 0.0));
        assert!(near(got, (1.5, 3.5)), "{got:?}");
    }

    #[test]
    fn scale_translate2d() {
        // The square is scaled to a half size of 2, mirrored, and the ball to a radius of 0.5.
        let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
        let a = ScaleTranslate2d::new(Vec2::new(-5.0, 0.0), 0.5);
        let b = ScaleTranslate2d::new(Vec2::ZERO, -2.0);
        let got = ball.at(&a).tttoiae(&square.at(&b), Vec2::X);
        assert!(near(got, (2.5, 7.5)), "{got:?}");
    }

    #[test]
    fn mixed_transformations() {
        // A ball placed by a `Vec2` against a square turned 45° by a `Transform2d`: it meets
        // the corner at x = -√2.
        let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
        let a = Vec2::new(-5.0, 0.0);
        let b = Transform2d::from_angle(std::f32::consts::FRAC_PI_4);
        let s = std::f32::consts::SQRT_2;
        let got = ball.at(&a).toiae(&square.at(&b), Vec2::X);
        assert!(near(got, (4.0 - s, 6.0 + s)), "{got:?}");
        let got = square.at(&b).toiae(&ball.at(&a), Vec2::NEG_X);
        assert!(near(got, (4.0 - s, 6.0 + s)), "{got:?}");
    }

    #[test]
    fn shape2d_time_travels() {
        let ball = Shape2d::ball(1.0);
        let square = Shape2d::square(1.0);
        let (a, b) = (Vec2::new(0.5, 0.0), Vec2::ZERO);
        let got = ball.at(&a).tttoiae(&square.at(&b), Vec2::X);
        assert!(near(got, (-2.5, 1.5)), "{got:?}");
        assert_eq!(ball.at(&a).tttoi(&square.at(&b), Vec2::X), got.map(|s| s.0));
    }

    #[test]
    fn poly2d_against_shape2d() {
        let square = unit_square();
        let point = Shape2d::point();
        let (a, b) = (Vec2::new(-4.0, 0.0), Vec2::ZERO);
        assert_eq!(
            square.at(&a).toiae(&point.at(&b), Vec2::X),
            Some((3.0, 5.0))
        );
        assert_eq!(
            point.at(&b).toiae(&square.at(&a), Vec2::NEG_X),
            Some((3.0, 5.0))
        );
    }

    #[cfg(feature = "bevy")]
    #[test]
    fn bevy_transform_mirrors_one_axis() {
        use bevy::prelude::Transform;
        // A right triangle with its right angle at the origin, mirrored in x: its long edge
        // faces the point coming from the left.
        let triangle = Poly2d::new(Arc::new(vec![
            Vec2::ZERO,
            Vec2::new(2.0, 0.0),
            Vec2::new(0.0, 2.0),
        ]));
        let tr = Transform::from_scale(auburn2d::Vec3::new(-1.0, 1.0, 1.0));
        let start = Vec2::new(-5.0, 0.5);
        let got = Point.at(&start).toiae(&triangle.at(&tr), Vec2::X);
        assert!(near(got, (3.5, 5.0)), "{got:?}");
    }

    #[cfg(feature = "bevy")]
    #[test]
    fn bevy_transform_stretches_a_ball() {
        // A unit ball stretched to an ellipse with half axes 2 and 1.
        use bevy::prelude::Transform;
        let ball = Ball::new(1.0);
        let tr = Transform::from_scale(auburn2d::Vec3::new(2.0, 1.0, 1.0));
        let got = Point
            .at(&Vec2::new(-5.0, 0.0))
            .toiae(&ball.at(&tr), Vec2::X);
        assert!(near(got, (3.0, 7.0)), "{got:?}");
        let got = Point
            .at(&Vec2::new(0.0, -5.0))
            .toiae(&ball.at(&tr), Vec2::Y);
        assert!(near(got, (4.0, 6.0)), "{got:?}");
    }

    /// A shape of the user's own, with only a support function.
    struct Diamond;

    impl Shape for Diamond {
        fn area(&self) -> f32 {
            2.0
        }
    }

    impl Support for Diamond {
        fn support(&self, dir: Vec2) -> Vec2 {
            if dir.x.abs() >= dir.y.abs() {
                Vec2::new(dir.x.signum(), 0.0)
            } else {
                Vec2::new(0.0, dir.y.signum())
            }
        }
    }

    #[test]
    fn a_shape_with_only_a_support_function() {
        let (a, b) = (Vec2::new(-5.0, 0.0), Vec2::ZERO);
        let got = Diamond.at(&a).toiae(&Box2d::square(1.0).at(&b), Vec2::X);
        assert!(near(got, (3.0, 7.0)), "{got:?}");
        let got = Ball::new(1.0).at(&b).tttoiae(&Diamond.at(&a), Vec2::NEG_X);
        assert!(near(got, (3.0, 7.0)), "{got:?}");
    }
}

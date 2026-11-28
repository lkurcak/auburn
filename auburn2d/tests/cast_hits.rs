//! `cast` hits worked out by hand: when the shapes first touch, the contact's normal, and a
//! point where they touch.

use auburn2d::prelude::*;
use std::f32::consts::FRAC_PI_4;
use std::sync::Arc;

#[track_caller]
fn assert_near(got: Vec2, want: Vec2) {
    assert!((got - want).length() < 1e-4, "got {got}, want {want}");
}

#[track_caller]
fn assert_hit(hit: Option<CastHit>, time: f32, normal: Vec2, point: Vec2) {
    let hit = hit.expect("a hit");
    assert!((hit.time - time).abs() < 1e-4, "{hit:?}: want time {time}");
    assert_near(hit.normal, normal);
    assert_near(hit.point, point);
}

fn floor() -> Box2d {
    // Its top is at y = 0, when it is at (0, -1).
    Box2d::new(Vec2::new(10.0, 1.0))
}

const FLOOR_AT: Vec2 = Vec2::new(0.0, -1.0);

/// Balls against boxes, which have exact formulas.
mod balls_and_boxes {
    use super::*;

    #[test]
    fn ball_meets_a_side() {
        let (ball, wall) = (Ball::new(1.0), Box2d::new(Vec2::new(1.0, 5.0)));
        let hit = ball
            .at(&Vec2::ZERO)
            .cast(&wall.at(&Vec2::new(4.0, 0.0)), Vec2::new(2.0, 1.0));
        assert_eq!(
            hit,
            Some(CastHit {
                time: 1.0,
                normal: Vec2::NEG_X,
                point: Vec2::new(3.0, 1.0),
            })
        );
    }

    #[test]
    fn ball_meets_a_corner() {
        // The ball meets the square's corner (2, 0.5) after travelling 2 - √0.75, and is
        // centred at (2 - √0.75, 0) then.
        let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
        let hit = ball
            .at(&Vec2::ZERO)
            .cast(&square.at(&Vec2::new(3.0, 1.5)), Vec2::new(2.0, 0.0));
        let s = 0.75f32.sqrt();
        assert_hit(
            hit,
            (2.0 - s) / 2.0,
            Vec2::new(-s, -0.5),
            Vec2::new(2.0, 0.5),
        );
    }

    #[test]
    fn box_meets_a_ball() {
        // The square's corner (1, 1) meets the ball after travelling 3 - √0.75. The normal is
        // out of the ball, the shape that doesn't move.
        let (square, ball) = (Box2d::square(1.0), Ball::new(1.0));
        let hit = square
            .at(&Vec2::ZERO)
            .cast(&ball.at(&Vec2::new(4.0, 1.5)), Vec2::X);
        let s = 0.75f32.sqrt();
        assert_hit(hit, 3.0 - s, Vec2::new(-s, -0.5), Vec2::new(4.0 - s, 1.0));
    }

    #[test]
    fn ball_in_a_box_leaves_through_its_nearest_side() {
        let (ball, square) = (Ball::new(0.5), Box2d::square(1.0));
        let hit = ball
            .at(&Vec2::new(0.8, 0.0))
            .cast(&square.at(&Vec2::ZERO), Vec2::new(-1.0, 3.0));
        assert_hit(hit, 0.0, Vec2::X, Vec2::new(1.0, 0.0));
    }

    #[test]
    fn ball_that_overlaps_a_corner() {
        // Its centre is 0.5 from the corner (1, 1), along the diagonal.
        let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
        let d = 0.5 * std::f32::consts::FRAC_1_SQRT_2;
        let hit = ball
            .at(&Vec2::splat(1.0 + d))
            .cast(&square.at(&Vec2::ZERO), Vec2::NEG_X);
        let diagonal = Vec2::splat(std::f32::consts::FRAC_1_SQRT_2);
        assert_hit(hit, 0.0, diagonal, Vec2::ONE);
    }
}

/// Boxes and polygons, by the separating axis theorem or, turned the same way, by slabs.
mod boxes_and_polygons {
    use super::*;

    #[test]
    fn box_lands_on_the_floor() {
        // Where the box's bottom side meets the floor, the middle of that side is the point.
        let crate_ = Box2d::new(Vec2::new(1.0, 0.5));
        let hit = crate_
            .at(&Vec2::new(2.0, 3.0))
            .cast(&floor().at(&FLOOR_AT), Vec2::new(0.0, -5.0));
        assert_eq!(
            hit,
            Some(CastHit {
                time: 0.5,
                normal: Vec2::Y,
                point: Vec2::new(2.0, 0.0),
            })
        );
    }

    #[test]
    fn box_on_the_floor_touches_it_whichever_way_it_moves() {
        // Touching counts as overlapping, so the hit is now, out of the floor, whether the box
        // slides along it, pushes into it or lifts off.
        let crate_ = Box2d::new(Vec2::new(1.0, 0.5));
        let (at, floor) = (Vec2::new(9.5, 0.5), floor());
        for vel in [
            Vec2::X,
            Vec2::new(3.0, -1.0),
            Vec2::new(-3.0, 1.0),
            Vec2::ZERO,
        ] {
            let hit = crate_.at(&at).cast(&floor.at(&FLOOR_AT), vel);
            // The box sticks out past the floor's end at x = 10.
            let want = CastHit {
                time: 0.0,
                normal: Vec2::Y,
                point: Vec2::new(9.25, 0.0),
            };
            assert_eq!(hit, Some(want), "{vel}");
        }
    }

    #[test]
    fn turned_box_lands_on_a_corner() {
        let square = Box2d::square(1.0);
        let at = Transform2d::from_translation(Vec2::new(0.0, 3.0)).with_angle(FRAC_PI_4);
        let hit = square
            .at(&at)
            .cast(&floor().at(&FLOOR_AT), Vec2::new(0.0, -2.0));
        let drop = 3.0 - std::f32::consts::SQRT_2;
        assert_hit(hit, drop / 2.0, Vec2::Y, Vec2::ZERO);
    }

    #[test]
    fn polygons_meet_side_to_side() {
        // The sides overlap from y = -0.5 to 1.
        let square = Poly2d::from(Box2d::square(1.0));
        let hit = square
            .at(&Vec2::new(-3.0, 0.0))
            .cast(&square.at(&Vec2::new(0.0, 0.5)), Vec2::X);
        assert_hit(hit, 1.0, Vec2::NEG_X, Vec2::new(-1.0, 0.25));
    }

    #[test]
    fn point_meets_a_slanted_side() {
        // The triangle's right side is on the line x + y = 1.
        let triangle = Poly2d::new(Arc::new(vec![
            Vec2::new(-1.0, 0.0),
            Vec2::new(1.0, 0.0),
            Vec2::new(0.0, 1.0),
        ]));
        let hit = Point
            .at(&Vec2::new(0.5, 3.0))
            .cast(&triangle.at(&Vec2::ZERO), Vec2::new(0.0, -2.0));
        let diagonal = Vec2::splat(std::f32::consts::FRAC_1_SQRT_2);
        assert_hit(hit, 1.25, diagonal, Vec2::splat(0.5));
    }

    #[test]
    fn point_in_a_polygon_leaves_through_its_nearest_side() {
        let square = Poly2d::from(Box2d::square(1.0));
        let hit = Point
            .at(&Vec2::new(0.25, -0.75))
            .cast(&square.at(&Vec2::ZERO), Vec2::X);
        assert_hit(hit, 0.0, Vec2::NEG_Y, Vec2::new(0.25, -1.0));
    }
}

/// A ball against a polygon goes through the shapes' support functions.
mod through_support_functions {
    use super::*;

    fn square() -> Poly2d {
        Poly2d::from(Box2d::square(1.0))
    }

    #[test]
    fn ball_lands_on_a_side() {
        let hit = Ball::new(1.0)
            .at(&Vec2::new(0.25, 3.0))
            .cast(&square().at(&Vec2::ZERO), Vec2::NEG_Y);
        assert_hit(hit, 1.0, Vec2::Y, Vec2::new(0.25, 1.0));
    }

    #[test]
    fn ball_meets_a_corner() {
        let hit = Ball::new(1.0)
            .at(&Vec2::ZERO)
            .cast(&square().at(&Vec2::new(3.0, 1.5)), Vec2::new(2.0, 0.0));
        let s = 0.75f32.sqrt();
        assert_hit(
            hit,
            (2.0 - s) / 2.0,
            Vec2::new(-s, -0.5),
            Vec2::new(2.0, 0.5),
        );
    }

    #[test]
    fn ball_on_a_side_touches_it_while_it_slides_or_pushes() {
        // Moving away, rounding could put the touch just in the past.
        let (ball, square) = (Ball::new(1.0), square());
        for vel in [Vec2::X, Vec2::new(1.0, -1.0)] {
            let hit = ball
                .at(&Vec2::new(0.5, 2.0))
                .cast(&square.at(&Vec2::ZERO), vel);
            assert_hit(hit, 0.0, Vec2::Y, Vec2::new(0.5, 1.0));
        }
    }

    #[test]
    fn ball_in_a_side_at_rest() {
        let hit = Ball::new(1.0)
            .at(&Vec2::new(0.5, 1.75))
            .cast(&square().at(&Vec2::ZERO), Vec2::ZERO);
        assert_hit(hit, 0.0, Vec2::Y, Vec2::new(0.5, 1.0));
    }

    #[test]
    fn ball_over_a_corner_at_rest() {
        // The ball's centre is 0.5 from the square's corner (1, 1), along the diagonal.
        let d = 0.5 * std::f32::consts::FRAC_1_SQRT_2;
        let hit = Ball::new(1.0)
            .at(&Vec2::splat(1.0 + d))
            .cast(&square().at(&Vec2::ZERO), Vec2::ZERO);
        let diagonal = Vec2::splat(std::f32::consts::FRAC_1_SQRT_2);
        assert_hit(hit, 0.0, diagonal, Vec2::ONE);
    }

    #[test]
    fn polygon_moves_out_of_a_ball() {
        // The normal is out of the ball, the shape that doesn't move.
        let hit = square()
            .at(&Vec2::new(0.0, -1.5))
            .cast(&Ball::new(1.0).at(&Vec2::ZERO), Vec2::NEG_Y);
        assert_hit(hit, 0.0, Vec2::NEG_Y, Vec2::new(0.0, -1.0));
    }
}

/// Points, and balls about the same centre, touch in every direction.
mod every_direction {
    use super::*;

    #[test]
    fn point_through_a_point_meets_it_head_on() {
        let hit = Point
            .at(&Vec2::new(-2.0, 0.0))
            .cast(&Point.at(&Vec2::ZERO), Vec2::new(4.0, 0.0));
        assert_hit(hit, 0.5, Vec2::NEG_X, Vec2::ZERO);
    }

    #[test]
    fn point_at_rest_on_a_point() {
        let hit = Point.at(&Vec2::ONE).cast(&Point.at(&Vec2::ONE), Vec2::ZERO);
        assert_hit(hit, 0.0, Vec2::X, Vec2::ONE);
    }

    #[test]
    fn ball_about_the_same_centre_backs_out() {
        let hit = Ball::new(1.0)
            .at(&Vec2::ZERO)
            .cast(&Ball::new(2.0).at(&Vec2::ZERO), Vec2::new(0.0, 3.0));
        assert_hit(hit, 0.0, Vec2::NEG_Y, Vec2::new(0.0, -2.0));
    }
}

#[test]
fn no_hit() {
    let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
    let (at, square_at) = (Vec2::ZERO, Vec2::new(4.0, 0.0));
    // It misses, it moves away, or it doesn't move.
    for vel in [Vec2::Y, Vec2::NEG_X, Vec2::ZERO] {
        let hit = ball.at(&at).cast(&square.at(&square_at), vel);
        assert_eq!(hit, None, "{vel}");
    }
}

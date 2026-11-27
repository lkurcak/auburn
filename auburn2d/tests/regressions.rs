//! Minimal reproducers for known bugs, grouped by the backlog item that fixes them.
//!
//! Each `#[ignore]` reason names the item's `BACKLOG.md` details file. The item un-ignores its
//! tests when it lands and moves them under "Fixed bugs". The randomized suite
//! (`tests/randomized`) covers the same bugs more broadly.

use auburn2d::prelude::*;
use auburn2d::transformation::Transformation2d;
use std::sync::Arc;

fn near(got: Option<f32>, want: f32) -> bool {
    got.is_some_and(|t| (t - want).abs() < 1e-3)
}

fn near_span(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
    got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-3 && (b - want.1).abs() < 1e-3)
}

fn square_poly(half: f32) -> Poly2d {
    Poly2d::from(Box2d::square(half))
}

// backlog/generic-shape-cast.md

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn ball_poly_cast_past_t_one() {
    let (a, b) = (Vec2::ZERO, Vec2::new(10.0, 0.0));
    let ball = Ball::new(1.0);
    // Ball against a Box2d of the same shape gives 8.
    assert!(near(
        ball.at(&a).toi(&Box2d::square(1.0).at(&b), Vec2::X),
        8.0
    ));
    let r = ball.at(&a).toi(&square_poly(1.0).at(&b), Vec2::X);
    assert!(near(r, 8.0), "{r:?}");
}

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn ball_poly_cast_hits_corner() {
    // The ball meets the square's corner (2, 0.5) after travelling 2 - √0.75.
    let (a, b, vel) = (Vec2::ZERO, Vec2::new(3.0, 1.5), Vec2::new(2.0, 0.0));
    let (ball, square) = (Ball::new(1.0), square_poly(1.0));
    let want = (2.0 - 0.75f32.sqrt()) / 2.0;
    let r = ball.at(&a).toi(&square.at(&b), vel);
    assert!(near(r, want), "toi {r:?}, want {want}");
    let r = ball.at(&a).tttoi(&square.at(&b), vel);
    assert!(near(r, want), "tttoi {r:?}, want {want}");
    let r = ball.at(&a).toiae(&square.at(&b), vel);
    assert!(
        r.is_some_and(|(entry, _)| (entry - want).abs() < 1e-3),
        "toiae {r:?}"
    );
}

// Fixed bugs.

/// `Poly2d::regular` went clockwise, and the Point–Poly2d casts at rest checked for a
/// counter-clockwise polygon, so a point at the centre of a hexagon wasn't in it. No `Poly2d`
/// query depends on the winding now, and `regular` goes counter-clockwise.
#[test]
fn point_at_regular_polygon_centre() {
    let hexagon = Poly2d::regular(6, 1.0);
    let z = Vec2::ZERO;
    assert!(Point.at(&z).collides(&hexagon.at(&z)));
    assert_eq!(Point.at(&z).toi(&hexagon.at(&z), Vec2::ZERO), Some(0.0));
    assert_eq!(Point.at(&z).toi(&hexagon.at(&z), Vec2::X), Some(0.0));
}

#[test]
fn regular_polygons_are_counter_clockwise() {
    let p = Poly2d::regular(6, 1.0).points;
    let twice_area: f32 = (0..p.len())
        .map(|i| p[i].perp_dot(p[(i + 1) % p.len()]))
        .sum();
    assert!(twice_area > 0.0, "{p:?}");
}

#[test]
fn clockwise_polygons_cast() {
    // The square [-1, 1]², clockwise, at (3, 0).
    let corners = [(-1.0, -1.0), (-1.0, 1.0), (1.0, 1.0), (1.0, -1.0)];
    let cw = Poly2d::new(Arc::new(corners.map(|(x, y)| Vec2::new(x, y)).to_vec()));
    let (z, c) = (Vec2::ZERO, Vec2::new(3.0, 0.0));
    assert_eq!(Point.at(&c).toi(&cw.at(&c), Vec2::ZERO), Some(0.0));
    assert!(near(Point.at(&z).toi(&cw.at(&c), Vec2::X), 2.0));
    let r = Box2d::square(1.0).at(&z).toi(&cw.at(&c), Vec2::X);
    assert!(near(r, 1.0), "box {r:?}");
    let r = cw.at(&z).toi(&square_poly(1.0).at(&c), Vec2::X);
    assert!(near(r, 1.0), "poly {r:?}");
    let t = Transform2d::from_translation(c);
    let r = Point.at(&t).tttoi(&cw.at(&t), Vec2::ZERO);
    assert_eq!(r, Some(f32::NEG_INFINITY), "Transform2d");
}

/// The Poly2d casts had absolute tolerances: a point 4e-4 outside a square of half size 1e-3
/// was in it, a velocity below 1e-5 was at rest, and one within about a degree of an edge of
/// a polygon of size 1e-4 ran along it. They have no tolerances now.
#[test]
fn small_polygon_excludes_nearby_point() {
    // A square of half size 1e-3, and a point 4e-4 outside its right edge.
    let square = square_poly(1e-3);
    let z = Vec2::ZERO;
    let p = Vec2::new(1.4e-3, 0.0);
    assert_eq!(Point.at(&p).toi(&square.at(&z), Vec2::ZERO), None);
}

#[test]
fn small_polygon_casts() {
    let (z, square) = (Vec2::ZERO, square_poly(1e-4));
    // 3e-4 to go at 5e-6.
    let r = Point
        .at(&z)
        .toi(&square.at(&Vec2::new(4e-4, 0.0)), Vec2::new(5e-6, 0.0));
    assert!(near(r, 60.0), "point {r:?}");
    // 5e-6 apart along y and closing at 4e-7, so they meet when x allows: from 80 to 120.
    let b = Vec2::new(1e-3, 2.05e-4);
    let r = square.at(&z).toiae(&square.at(&b), Vec2::new(1e-5, 4e-7));
    assert!(near_span(r, (80.0, 120.0)), "poly {r:?}");
}

/// `Poly2d::area` was `unimplemented!()`.
#[test]
fn poly2d_area() {
    assert!((square_poly(1.0).area() - 4.0).abs() < 1e-6);
    let hexagon = 1.5 * 3f32.sqrt();
    assert!((Poly2d::regular(6, 1.0).area() - hexagon).abs() < 1e-5);
    let mut far = (*square_poly(1.0).points).clone();
    far.reverse();
    let far = far.into_iter().map(|p| p + Vec2::new(1e4, -1e4)).collect();
    assert!((Poly2d::new(Arc::new(far)).area() - 4.0).abs() < 1e-6);
}

/// An empty `Poly2d`, as from `Default`, panicked in `support`. It is a point at its origin
/// now, like a polygon with one vertex there.
#[test]
fn empty_poly2d_is_a_point_at_its_origin() {
    let empty = Poly2d::default();
    let (z, c) = (Vec2::ZERO, Vec2::new(3.0, 0.0));
    assert_eq!(empty.area(), 0.0);
    assert!(!Point.at(&z).collides(&empty.at(&c)));
    assert!(
        Ball::new(1.0)
            .at(&Vec2::new(2.5, 0.0))
            .collides(&empty.at(&c))
    );
    assert!(near(Point.at(&z).toi(&empty.at(&c), Vec2::X), 3.0));
    let r = Box2d::square(1.0).at(&z).toi(&empty.at(&c), Vec2::X);
    assert!(near(r, 2.0), "box {r:?}");
    let r = empty.at(&c).toi(&square_poly(1.0).at(&z), Vec2::NEG_X);
    assert!(near(r, 2.0), "poly {r:?}");
    let (tz, tc) = (Transform2d::IDENTITY, Transform2d::from_translation(c));
    let r = Point.at(&tz).toi(&empty.at(&tc), Vec2::X);
    assert!(near(r, 3.0), "Transform2d {r:?}");
}

/// `Poly2d::new` took any points. Debug builds check that they go once around a convex
/// polygon now.
#[test]
#[cfg(debug_assertions)]
fn poly2d_new_checks_convexity_in_debug_builds() {
    let panics =
        |points: Vec<Vec2>| std::panic::catch_unwind(|| Poly2d::new(Arc::new(points))).is_err();
    let arrowhead = [(0.0, 2.0), (-1.0, 0.0), (0.0, 0.5), (1.0, 0.0)];
    assert!(panics(arrowhead.map(|(x, y)| Vec2::new(x, y)).to_vec()));
    let pentagon = Poly2d::regular(5, 1.0).points;
    assert!(
        panics([0, 2, 4, 1, 3].map(|i| pentagon[i]).to_vec()),
        "star"
    );
    // Either way round, nearly flat, on a line, or fewer than three points.
    let mut cw = (*square_poly(1.0).points).clone();
    cw.reverse();
    assert!(!panics(cw));
    assert!(!panics((*Poly2d::regular(1000, 1e-3).points).clone()));
    assert!(!panics(vec![Vec2::ZERO, Vec2::X, 2.0 * Vec2::X]));
    assert!(!panics(vec![Vec2::ZERO, Vec2::X]));
    assert!(!panics(vec![]));
}

/// Point–Ball and Ball–Ball `tttoi` and `tttoiae` divided by a zero velocity and gave NaN.
/// At rest, overlapping shapes now overlap forever, as the `relation::cast` docs say.
#[test]
fn time_traveling_casts_at_rest_are_not_nan() {
    let (p, inside, far) = (Vec2::ZERO, Vec2::new(0.5, 0.0), Vec2::new(5.0, 0.0));
    let ball = Ball::new(1.0);
    let forever = Some((f32::NEG_INFINITY, f32::INFINITY));
    let r = Point.at(&p).tttoi(&ball.at(&inside), Vec2::ZERO);
    assert_eq!(r, Some(f32::NEG_INFINITY), "overlapping");
    assert_eq!(
        Point.at(&p).tttoiae(&ball.at(&inside), -Vec2::ZERO),
        forever
    );
    assert_eq!(
        ball.at(&p).toiae(&ball.at(&inside), Vec2::ZERO),
        Some((0.0, f32::INFINITY))
    );
    assert_eq!(Point.at(&p).tttoi(&ball.at(&far), Vec2::ZERO), None);
    assert_eq!(ball.at(&p).tttoiae(&ball.at(&far), Vec2::ZERO), None);
    let (t1, t2) = (
        Transform2d::from_translation(p),
        Transform2d::from_translation(inside),
    );
    let r = ball.at(&t1).tttoi(&ball.at(&t2), Vec2::ZERO);
    assert_eq!(r, Some(f32::NEG_INFINITY), "Transform2d");
}

/// Point–Point casts were always `None`. Points touch where they coincide, as with `collides`.
#[test]
fn point_point_casts() {
    let (a, b) = (Vec2::new(1.0, 2.0), Vec2::new(4.0, 6.0));
    // Straight at b, which is 5 away along (0.6, 0.8).
    let vel = Vec2::new(0.6, 0.8);
    assert!(near(Point.at(&a).toi(&Point.at(&b), vel), 5.0));
    assert!(near_span(
        Point.at(&a).toiae(&Point.at(&b), vel),
        (5.0, 5.0)
    ));
    assert!(near(Point.at(&a).tttoi(&Point.at(&b), -vel), -5.0));
    assert_eq!(Point.at(&a).toi(&Point.at(&b), -vel), None, "b is behind");
    assert_eq!(Point.at(&a).tttoi(&Point.at(&b), Vec2::X), None, "missing");
    assert_eq!(Point.at(&a).toi(&Point.at(&b), Vec2::ZERO), None);
    assert_eq!(Point.at(&a).toi(&Point.at(&a), Vec2::ZERO), Some(0.0));
    let (ta, tb) = (
        Transform2d::from_translation(a),
        Transform2d::from_translation(b),
    );
    assert!(near(Point.at(&ta).toi(&Point.at(&tb), vel), 5.0));
}

/// `Transform2d` ball casts scaled radii twice, by the wrong collider's scale, or by a
/// negative scale, and `toiae` gave no exit when starting inside. They now cast the
/// world-space balls, of radius `r * |scale|`, with the exact `Vec2` casts.
#[test]
fn transform2d_ball_ball_scale_applies_once() {
    // Ball(1) at 0 against Ball(1) scaled by 2 at (5, 0): a gap of 2.
    let a = Transform2d::from_translation(Vec2::ZERO);
    let b = Transform2d::from_translation(Vec2::new(5.0, 0.0)).with_scale(2.0);
    let ball = Ball::new(1.0);
    assert!(!ball.at(&a).collides(&ball.at(&b)));
    assert!(near(ball.at(&a).toi(&ball.at(&b), Vec2::X), 2.0));
}

#[test]
fn transform2d_ball_point_uses_the_ball_scale() {
    // Ball(1) scaled by 2 at 0, point at (3, 0) with its own scale.
    let ball = Transform2d::from_translation(Vec2::ZERO).with_scale(2.0);
    let point = Transform2d::from_translation(Vec2::new(3.0, 0.0)).with_scale(0.5);
    let r = Ball::new(1.0).at(&ball).toi(&Point.at(&point), Vec2::X);
    assert!(near(r, 1.0), "{r:?}");
    let r = Point.at(&point).toi(&Ball::new(1.0).at(&ball), Vec2::NEG_X);
    assert!(near(r, 1.0), "{r:?}");
}

#[test]
fn transform2d_toiae_exits_when_starting_inside() {
    let t = Transform2d::from_translation(Vec2::ZERO);
    let r = Point.at(&t).toiae(&Ball::new(1.0).at(&t), Vec2::X);
    assert!(near_span(r, (0.0, 1.0)), "{r:?}");
}

#[test]
fn transform2d_negative_scale_casts() {
    // Scale -1 at 0, the other at (5, 0): a gap of 3.
    let a = Transform2d::from_translation(Vec2::ZERO).with_scale(-1.0);
    let b = Transform2d::from_translation(Vec2::new(5.0, 0.0));
    let (ball, square) = (Ball::new(1.0), Box2d::square(1.0));
    let r = ball.at(&a).toi(&ball.at(&b), Vec2::X);
    assert!(near(r, 3.0), "ball-ball {r:?}");
    let r = square.at(&a).toi(&square.at(&b), Vec2::X);
    assert!(near(r, 3.0), "box-box {r:?}");
    let r = ball.at(&a).toi(&square.at(&b), Vec2::X);
    assert!(near(r, 3.0), "ball-box {r:?}");
}

/// EPA refined a near-circle evenly, so it ran out of iterations before the tiny differences
/// between directions showed, and returned its last edge, pointing anywhere. It now returns
/// the shortest support distance it has seen.
#[test]
fn deep_near_concentric_balls_separate() {
    let (a, b) = (Vec2::new(0.1655, 0.1422), Vec2::ZERO);
    let (small, large) = (Ball::new(0.7204), Ball::new(4.7942));
    let p = small.at(&a).penetrates(&large.at(&b)).unwrap();
    let depth = (0.7204 + 4.7942) - (a - b).length();
    let left = (0.7204 + 4.7942) - (a + p - b).length();
    assert!(
        (p.length() - depth).abs() < 1e-3 && left.abs() < 1e-3,
        "p = {p:?} for a depth of {depth} leaves {left} of overlap"
    );
}

#[test]
fn concentric_balls_penetrate_by_full_depth() {
    let z = Vec2::ZERO;
    let p = Ball::new(1.0)
        .at(&z)
        .penetrates(&Ball::new(2.0).at(&z))
        .unwrap();
    assert!((p.length() - 3.0).abs() < 1e-3, "{p:?}");
}

#[test]
fn coincident_boxes_penetrate_by_full_depth() {
    let z = Vec2::ZERO;
    let rect = Box2d::new(Vec2::new(2.0, 1.0));
    let p = rect.at(&z).penetrates(&rect.at(&z)).unwrap();
    assert!((p.length() - 2.0).abs() < 1e-3, "{p:?}");
}

/// GJK compared a cubic triple product with an absolute `1e-10`, so small shapes collided
/// across a gap, depending on the order.
#[test]
fn small_balls_apart_in_both_directions() {
    // 0.022 apart with radii 0.01: a gap of 0.002.
    let ball = Ball::new(0.01);
    let (a, b) = (Vec2::new(0.00123, -0.0007), Vec2::new(0.016786, -0.016256));
    assert!(!ball.at(&a).collides(&ball.at(&b)));
    assert!(!ball.at(&b).collides(&ball.at(&a)));
}

#[test]
fn no_false_positives_at_small_scale() {
    // Balls with a gap of a tenth of their radius, in 360 directions.
    for r in [0.01f32, 0.001] {
        for i in 0..360 {
            let dir = Rotor2d::radians((i as f32).to_radians()) * Vec2::X;
            let a = Vec2::new(0.123 * r, -0.07 * r);
            let b = a + dir * (2.2 * r);
            let ball = Ball::new(r);
            assert!(!ball.at(&a).collides(&ball.at(&b)), "r = {r}, {i}°");
        }
    }
}

/// The same scenes at every scale give the same answers, scaled.
#[test]
fn scale_invariance() {
    // Rotated and scaled, so `Collider::support` maps every direction.
    let placed = |p: Vec2| {
        Transform2d::from_translation(p)
            .with_rotation(Rotor2d::radians(0.3))
            .with_scale(2.0)
    };
    for k in [1e-6f32, 1e-3, 1.0, 1e3, 1e6] {
        let (ball, square) = (Ball::new(k), Box2d::square(k));
        let z = Vec2::ZERO;
        for i in 0..24 {
            let dir = Rotor2d::radians((15.0 * i as f32).to_radians()) * Vec2::X;
            let (apart, overlapping) = (dir * (2.2 * k), dir * (1.8 * k));
            let msg = format!("k = {k}, {i}");
            assert!(!ball.at(&z).collides(&ball.at(&apart)), "{msg}");
            assert!(!ball.at(&apart).collides(&ball.at(&z)), "{msg}");
            assert!(ball.at(&z).collides(&ball.at(&overlapping)), "{msg}");
            // Scaled by 2, so 0.4k apart and overlapping by 0.4k.
            let c = Vec2::new(0.5 * k, -0.25 * k);
            let (pa, pb) = (placed(c), placed(c + dir * (4.4 * k)));
            assert!(!ball.at(&pa).collides(&ball.at(&pb)), "{msg}");
            assert!(!ball.at(&pb).collides(&ball.at(&pa)), "{msg}");
            assert!(
                ball.at(&pa)
                    .collides(&ball.at(&placed(c + dir * (3.6 * k)))),
                "{msg}"
            );
        }
        // Squares overlapping by 0.2k along x, and 0.2k apart.
        let p = square
            .at(&z)
            .penetrates(&square.at(&Vec2::new(1.8 * k, 0.3 * k)));
        assert!(
            p.is_some_and(|p| (p - Vec2::new(-0.2 * k, 0.0)).length() < 1e-4 * k),
            "k = {k}: {p:?}"
        );
        let apart = Vec2::new(2.2 * k, 0.3 * k);
        assert!(!square.at(&z).collides(&square.at(&apart)), "k = {k}");
    }
}

/// Coincident points touch, as they do in 3D.
#[test]
fn coincident_points_touch() {
    for p in [Vec2::ZERO, Vec2::new(3.0, -2.0)] {
        assert!(Point.at(&p).collides(&Point.at(&p)), "{p}");
        assert_eq!(
            Point.at(&p).penetrates(&Point.at(&p)),
            Some(Vec2::ZERO),
            "{p}"
        );
        let dot = Ball::new(0.0);
        assert!(dot.at(&p).collides(&dot.at(&p)), "{p}");
        assert!(Point.at(&p).collides(&dot.at(&p)), "{p}");
        assert!(!Point.at(&p).collides(&Point.at(&(p + Vec2::X))), "{p}");
    }
}

/// Far from the origin, `Collider::support` blurred GJK's short search directions. They are
/// now as long as the shapes, which is enough at 1e4.
#[test]
fn far_from_origin_ball_contains_point() {
    // 0.69 apart with radius 0.94, near (1e4, -1e4).
    let ball = Ball::new(0.942_961_9);
    let (p, c) = (
        Vec2::new(9997.541, -9997.661),
        Vec2::new(9998.233, -9997.66),
    );
    assert!(Point.at(&p).collides(&ball.at(&c)));
    assert!(ball.at(&c).collides(&Point.at(&p)));
    assert!(ball.at(&c).penetrates(&Point.at(&p)).is_some());
}

/// `Collider::support` mapped directions through the inverse transformation, adding and then
/// subtracting its translation, so far from the origin short directions came out as zero.
#[test]
fn far_from_origin_support_keeps_short_directions() {
    use auburn2d::transformation::pos2d::Pos2d;
    let c = Vec2::new(1e4, -1e4);
    let ball = Ball::new(1.0);
    for len in [1e-4f32, 1e-8] {
        let d = Vec2::X * len;
        let want = c + Vec2::X;
        let got = [
            ball.at(&c).support(d),
            ball.at(&Pos2d(c)).support(d),
            ball.at(&ScaleTranslate2d::from_translation(c)).support(d),
            ball.at(&Transform2d::from_translation(c)).support(d),
        ];
        for (i, s) in got.into_iter().enumerate() {
            assert!((s - want).length() < 1e-2, "{len}, {i}: {s}");
        }
    }

    // Turned and scaled by 2.
    let t = Transform2d::from_translation(c)
        .with_angle(0.3)
        .with_scale(2.0);
    for i in 0..8 {
        let d = Rotor2d::radians(i as f32 * 0.8) * Vec2::X * 1e-4;
        let s = ball.at(&t).support(d);
        let want = c + 2.0 * d.normalize();
        assert!((s - want).length() < 1e-2, "ball {i}: {s}, want {want}");
        let corners = [
            Vec2::ONE,
            Vec2::new(1.0, -1.0),
            Vec2::NEG_ONE,
            Vec2::new(-1.0, 1.0),
        ];
        let corner = corners
            .into_iter()
            .max_by(|a, b| (t.rot * *a).dot(d).total_cmp(&(t.rot * *b).dot(d)))
            .unwrap();
        let (s, want) = (Box2d::square(1.0).at(&t).support(d), t.apply(corner));
        assert!((s - want).length() < 1e-2, "square {i}: {s}, want {want}");
    }
}

/// A user-defined transformation with uneven scale, which `Transformation2d` allows.
#[derive(Debug, Clone, Copy)]
struct Stretch {
    t: Vec2,
    s: Vec2,
}

impl Transformation2d for Stretch {
    fn apply_to_origin(&self) -> Vec2 {
        self.t
    }
    fn apply(&self, p: Vec2) -> Vec2 {
        p * self.s + self.t
    }
    fn inverse(&self) -> Self {
        Stretch {
            t: -self.t / self.s,
            s: 1.0 / self.s,
        }
    }
    fn compose(&self, o: &Self) -> Self {
        Stretch {
            t: self.s * o.t + self.t,
            s: self.s * o.s,
        }
    }
    fn support_direction(&self, dir: Vec2) -> Vec2 {
        dir * self.s
    }
}

/// `Collider::support` went through `inverse`, which is right only for uniform scale. Such
/// types now implement `Transformation2d::support_direction`.
#[test]
fn uneven_scale_support() {
    // Ball(1) stretched to an ellipse with semi-axes 1 and 4. Its support along (1, 1) is
    // the point maximising x + y.
    let e = Stretch {
        t: Vec2::ZERO,
        s: Vec2::new(1.0, 4.0),
    };
    let s = Ball::new(1.0).at(&e).support(Vec2::ONE);
    let want = Vec2::new(1.0, 16.0) / 17f32.sqrt();
    assert!((s - want).length() < 1e-4, "{s:?}, want {want:?}");
}

// backlog/bevy-uneven-scale.md

#[cfg(feature = "bevy")]
mod bevy_transform {
    use super::*;
    use bevy::prelude::{Transform, Vec3};
    use std::sync::Arc;

    #[test]
    #[ignore = "backlog/bevy-uneven-scale.md"]
    fn uneven_scale_box() {
        // Box2d(1) under scale (1, 3, 1) covers x in [-1, 1] and y in [-3, 3].
        let t = Transform::from_scale(Vec3::new(1.0, 3.0, 1.0));
        let inside = Transform::from_xyz(0.0, 2.5, 0.0);
        let outside = Transform::from_xyz(2.5, 0.0, 0.0);
        assert!(Point.at(&inside).collides(&Box2d::square(1.0).at(&t)));
        assert!(!Point.at(&outside).collides(&Box2d::square(1.0).at(&t)));
    }

    #[test]
    #[ignore = "backlog/bevy-uneven-scale.md"]
    fn z_scale_does_not_leak_into_2d() {
        let t = Transform::from_scale(Vec3::new(1.0, 1.0, 5.0));
        let outside = Transform::from_xyz(2.5, 0.0, 0.0);
        assert!(!Point.at(&outside).collides(&Box2d::square(1.0).at(&t)));
    }

    #[test]
    #[ignore = "backlog/bevy-uneven-scale.md"]
    fn mirrored_triangle() {
        let triangle = Poly2d::new(Arc::new(vec![
            Vec2::new(0.0, 0.0),
            Vec2::new(2.0, 0.0),
            Vec2::new(0.0, 1.0),
        ]));
        let t = Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
        let inside = Transform::from_xyz(-1.5, 0.2, 0.0);
        assert!(Point.at(&inside).collides(&triangle.at(&t)));
    }
}

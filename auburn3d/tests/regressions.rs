//! Minimal reproducers for known bugs, grouped by the backlog item that fixes them.
//!
//! Each `#[ignore]` reason names the item's `BACKLOG.md` details file. The item un-ignores its
//! tests when it lands and moves them under "Fixed bugs". The randomized suite
//! (`tests/randomized`) covers the same bugs more broadly.

use auburn3d::prelude::*;

fn near(got: Option<f32>, want: f32) -> bool {
    got.is_some_and(|t| (t - want).abs() < 1e-3)
}

fn near_span(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
    got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-3 && (b - want.1).abs() < 1e-3)
}

const X5: Vec3 = Vec3::new(5.0, 0.0, 0.0);

// backlog/epa-3d.md

#[test]
#[ignore = "backlog/epa-3d.md"]
fn ball_point_penetration_depth() {
    let (a, b) = (
        Vec3::new(2.008_430_5, 1.023_615_4, -2.408_577_4),
        Vec3::new(2.746_634_5, 0.946_395_9, -2.165_105_3),
    );
    let p = Ball::new(1.804_701_3).at(&a).penetrates(&Point.at(&b));
    assert!(
        p.is_some_and(|p| (p.length() - 1.02356).abs() < 1e-2),
        "{p:?}"
    );
}

#[test]
#[ignore = "backlog/epa-3d.md"]
fn box_box_penetration_depth() {
    let (a, b) = (
        Vec3::new(-0.052_579_165, 2.863_538_7, 2.765_394_7),
        Vec3::new(-0.422_937_87, 2.543_839_5, 1.649_674_9),
    );
    let ba = Box3d::new(Vec3::new(1.278_554_9, 1.694_450_7, 0.182_445_94));
    let bb = Box3d::new(Vec3::new(0.850_196_7, 0.184_416_8, 1.903_317_3));
    let p = bb.at(&b).penetrates(&ba.at(&a));
    assert!(
        p.is_some_and(|p| (p.length() - 0.97004).abs() < 1e-2),
        "{p:?}"
    );
}

#[test]
#[ignore = "backlog/epa-3d.md"]
fn ball_ball_penetration() {
    let (a, b) = (
        Vec3::new(-1.640_698_3, -0.115_837_81, 2.472_049_7),
        Vec3::new(-2.496_988_8, -1.151_876_4, 0.250_179_77),
    );
    let p = Ball::new(1.301_190_9)
        .at(&a)
        .penetrates(&Ball::new(1.995_153_3).at(&b));
    assert!(
        p.is_some_and(|p| (p.length() - 0.69955).abs() < 1e-2),
        "{p:?}"
    );
    let z = Vec3::ZERO;
    let p = Ball::new(1.0)
        .at(&z)
        .penetrates(&Ball::new(1.0).at(&Vec3::X));
    assert!(
        p.is_some_and(|p| (p - Vec3::NEG_X).length() < 1e-2),
        "{p:?}"
    );
    let p = Ball::new(1.0).at(&z).penetrates(&Ball::new(2.0).at(&z));
    assert!(
        p.is_some_and(|p| (p.length() - 3.0).abs() < 2e-2),
        "concentric {p:?}"
    );
}

// backlog/cast-edge-cases.md

#[test]
#[ignore = "backlog/cast-edge-cases.md"]
fn time_traveling_casts_go_negative() {
    let z = Vec3::ZERO;
    let ball = Ball::new(1.0);
    let r = ball
        .at(&z)
        .tttoi(&Point.at(&Vec3::new(0.5, 0.0, 0.0)), Vec3::X);
    assert!(near(r, -0.5), "{r:?}");
    let r = ball
        .at(&z)
        .tttoiae(&Point.at(&Vec3::new(-3.0, 0.0, 0.0)), Vec3::X);
    assert!(near_span(r, (-4.0, -2.0)), "contact in the past: {r:?}");
}

#[test]
#[ignore = "backlog/cast-edge-cases.md"]
fn overlap_at_rest() {
    let z = Vec3::ZERO;
    let ball = Ball::new(1.0);
    let r = ball
        .at(&z)
        .toi(&Point.at(&Vec3::new(0.5, 0.0, 0.0)), Vec3::ZERO);
    assert_eq!(r, Some(0.0));
    let r = ball.at(&z).toi(&ball.at(&Vec3::X), Vec3::ZERO);
    assert_eq!(r, Some(0.0));
}

#[test]
#[ignore = "backlog/cast-edge-cases.md"]
fn slow_cast() {
    let z = Vec3::ZERO;
    let ball = Ball::new(1.0);
    let r = ball.at(&z).toi(
        &ball.at(&Vec3::new(3.0, 0.0, 0.0)),
        Vec3::new(1e-6, 0.0, 0.0),
    );
    assert!(r.is_some_and(|t| (t - 1e6).abs() < 1e3), "{r:?}");
}

// backlog/generic-shape-cast.md

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn box_casts_past_t_one() {
    let z = Vec3::ZERO;
    let cube = Box3d::cube(1.0);
    let r = cube
        .at(&z)
        .toi(&cube.at(&Vec3::new(10.0, 0.0, 0.0)), Vec3::X);
    assert!(near(r, 8.0), "toi {r:?}");
    let r = cube.at(&z).toiae(
        &cube.at(&Vec3::new(3.0, 0.0, 0.0)),
        Vec3::new(2.0, 0.0, 0.0),
    );
    assert!(near_span(r, (0.5, 2.5)), "toiae {r:?}");
}

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn box_cast_does_not_stall() {
    let (a, b) = (
        Vec3::new(0.734_034_06, -1.500_807_9, -0.983_083_25),
        Vec3::new(-2.359_465, 2.231_857_3, -2.132_815_8),
    );
    let ba = Box3d::new(Vec3::new(1.888_601_7, 1.540_556_1, 1.050_958_3));
    let bb = Box3d::new(Vec3::new(1.635_752_7, 1.587_372, 1.962_005_4));
    let vel = Vec3::new(-8.827_759, 8.827_759, -8.827_759);
    let r = ba.at(&a).toi(&bb.at(&b), vel);
    assert!(near(r, 0.0685), "{r:?}");
}

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn box_ball_toiae() {
    let r = Box3d::cube(1.0)
        .at(&Vec3::ZERO)
        .toiae(&Ball::new(1.0).at(&X5), X5);
    assert!(near_span(r, (0.6, 1.4)), "{r:?}");
}

// Fixed bugs.

#[test]
/// GJK exited early on an absolute `first_point.length_squared() < 1e-10`.
fn tiny_shapes_apart() {
    let z = Vec3::ZERO;
    // A gap of 8e-6.
    let ball = Ball::new(1e-6);
    assert!(!ball.at(&z).collides(&ball.at(&Vec3::new(1e-5, 0.0, 0.0))));
    assert!(!Point.at(&z).collides(&Point.at(&Vec3::new(5e-6, 0.0, 0.0))));
    // A gap of 3e-6.
    let cube = Box3d::cube(1e-6);
    assert!(!cube.at(&z).collides(&cube.at(&Vec3::new(0.0, 0.0, 5e-6))));
}

/// The same scenes at every scale give the same answers, scaled.
#[test]
fn scale_invariance() {
    // Rotated and scaled, so `Collider::support` maps every direction.
    let placed = |p: Vec3| {
        let axis = Vec3::new(1.0, 2.0, 3.0).normalize();
        Transform3d::from_rotation(auburn3d::Quat::from_axis_angle(axis, 0.7))
            .with_translation(p)
            .with_scale(2.0)
    };
    let dirs = [
        Vec3::X,
        Vec3::NEG_Y,
        Vec3::new(1.0, 1.0, 1.0).normalize(),
        Vec3::new(-0.3, 0.8, -0.5).normalize(),
    ];
    for k in [1e-6f32, 1e-3, 1.0, 1e3, 1e6] {
        let (ball, cube) = (Ball::new(k), Box3d::cube(k));
        let z = Vec3::ZERO;
        for (i, dir) in dirs.into_iter().enumerate() {
            let (apart, overlapping) = (dir * (2.2 * k), dir * (1.8 * k));
            let msg = format!("k = {k}, {i}");
            assert!(!ball.at(&z).collides(&ball.at(&apart)), "{msg}");
            assert!(!ball.at(&apart).collides(&ball.at(&z)), "{msg}");
            assert!(ball.at(&z).collides(&ball.at(&overlapping)), "{msg}");
            // Scaled by 2, so 0.4k apart and overlapping by 0.4k.
            let c = Vec3::new(0.5 * k, -0.25 * k, 0.75 * k);
            let (pa, pb) = (placed(c), placed(c + dir * (4.4 * k)));
            assert!(!ball.at(&pa).collides(&ball.at(&pb)), "{msg}");
            assert!(!ball.at(&pb).collides(&ball.at(&pa)), "{msg}");
            assert!(
                ball.at(&pa)
                    .collides(&ball.at(&placed(c + dir * (3.6 * k)))),
                "{msg}"
            );
        }
        // Cubes overlapping by 0.2k along x, and 0.2k apart.
        let p = cube
            .at(&z)
            .penetrates(&cube.at(&Vec3::new(1.8 * k, 0.3 * k, -0.4 * k)));
        assert!(
            p.is_some_and(|p| (p - Vec3::new(-0.2 * k, 0.0, 0.0)).length() < 1e-4 * k),
            "k = {k}: {p:?}"
        );
        let apart = Vec3::new(2.2 * k, 0.3 * k, -0.4 * k);
        assert!(!cube.at(&z).collides(&cube.at(&apart)), "k = {k}");
    }
}

/// Coincident points touch, as they do in 2D.
#[test]
fn coincident_points_touch() {
    for p in [Vec3::ZERO, Vec3::new(3.0, -2.0, 1.0)] {
        assert!(Point.at(&p).collides(&Point.at(&p)), "{p}");
        assert_eq!(
            Point.at(&p).penetrates(&Point.at(&p)),
            Some(Vec3::ZERO),
            "{p}"
        );
        let dot = Ball::new(0.0);
        assert!(dot.at(&p).collides(&dot.at(&p)), "{p}");
        assert!(Point.at(&p).collides(&dot.at(&p)), "{p}");
        assert!(!Point.at(&p).collides(&Point.at(&(p + Vec3::X))), "{p}");
    }
}

/// The reversed 3D casts swapped entry and exit.
#[test]
fn reversed_toiae_keeps_entry_first() {
    let z = Vec3::ZERO;
    let ball = Ball::new(1.0);
    let r = Point.at(&z).toiae(&ball.at(&X5), Vec3::X);
    assert!(near_span(r, (4.0, 6.0)), "point-ball {r:?}");
    let r = Point.at(&z).tttoiae(&ball.at(&X5), Vec3::X);
    assert!(near_span(r, (4.0, 6.0)), "point-ball time-traveling {r:?}");
    let r = ball.at(&z).toiae(&ball.at(&X5), Vec3::X);
    assert!(near_span(r, (3.0, 7.0)), "ball-ball {r:?}");
}

// Degenerate shapes that work today.

#[test]
fn flat_boxes() {
    let z = Vec3::ZERO;
    let flat = Box3d::new(Vec3::new(1.0, 1.0, 0.0));
    assert!(flat.at(&z).collides(&flat.at(&Vec3::new(0.5, 0.5, 0.0))));
    assert!(Point.at(&z).collides(&flat.at(&z)));
}

// backlog/bevy-uneven-scale.md

#[cfg(feature = "bevy")]
mod bevy_transform {
    use super::*;
    use bevy::prelude::Transform;

    #[test]
    #[ignore = "backlog/bevy-uneven-scale.md"]
    fn uneven_scale_ball() {
        // Ball(1) under scale (1, 4, 1): an ellipsoid. Its support along (1, 1, 0) is the
        // point maximising x + y.
        let t = Transform::from_scale(Vec3::new(1.0, 4.0, 1.0));
        let ball = Ball::new(1.0);
        let s = ball.at(&t).support(Vec3::new(1.0, 1.0, 0.0));
        let want = Vec3::new(1.0, 16.0, 0.0) / 17f32.sqrt();
        assert!((s - want).length() < 1e-3, "{s:?}, want {want:?}");
        // 0.36 + 0.36 < 1 is inside; 0.64 + 0.4225 > 1 is outside.
        assert!(Point.at(&Vec3::new(0.6, 2.4, 0.0)).collides(&ball.at(&t)));
        assert!(!Point.at(&Vec3::new(0.8, 2.6, 0.0)).collides(&ball.at(&t)));
    }

    #[test]
    #[ignore = "backlog/bevy-uneven-scale.md"]
    fn mirrored_box() {
        let t = Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0));
        let cube = Box3d::cube(1.0);
        assert!(Point.at(&Vec3::new(0.5, 0.0, 0.0)).collides(&cube.at(&t)));
        let s = cube.at(&t).support(Vec3::X);
        assert!((s.x - 1.0).abs() < 1e-5, "{s:?}");
    }

    #[test]
    fn uneven_scale_box() {
        let t = Transform::from_scale(Vec3::new(1.0, 4.0, 1.0));
        assert!(
            Point
                .at(&Vec3::new(0.5, 3.5, 0.0))
                .collides(&Box3d::cube(1.0).at(&t))
        );
    }
}

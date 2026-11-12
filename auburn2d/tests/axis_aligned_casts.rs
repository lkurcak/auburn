//! Casts straight along an axis, where the slab method divides by a zero velocity component.
//!
//! A cast whose path runs exactly along one of a box's edge lines (or exactly a radius away
//! from one) used to compute `0 * inf = NaN` and miss. Every case here is checked for each cast
//! function, both argument orders, `Vec2` and `Transform2d` positions, all four directions and
//! both signs of zero.

use auburn2d::prelude::*;

const EPS: f32 = 1e-5;

/// The box every case casts at: wider than tall, so swapped axes show.
const HALF: Vec2 = Vec2::new(1.0, 0.5);

/// Where the mover starts and how fast it goes, in the frame where it moves along +y. It
/// starts at y = -3 with speed 1, so times are distances.
const START_Y: f32 = -3.0;

/// Maps the canonical frame (moving along +y) to another.
type Frame = fn(Vec2) -> Vec2;

/// The ways to point a cast along an axis: a function from the canonical frame (moving along
/// +y) to the actual one, which is a rotation by a multiple of 90° or a mirror. Each zero
/// component comes out with both signs across the list.
fn frames() -> Vec<(&'static str, Frame)> {
    vec![
        ("+y", |v| v),
        ("+y mirrored", |v| Vec2::new(-v.x, v.y)),
        ("-y", |v| Vec2::new(v.x, -v.y)),
        ("-y mirrored", |v| Vec2::new(-v.x, -v.y)),
        ("+x", |v| Vec2::new(v.y, v.x)),
        ("+x mirrored", |v| Vec2::new(v.y, -v.x)),
        ("-x", |v| Vec2::new(-v.y, v.x)),
        ("-x mirrored", |v| Vec2::new(-v.y, -v.x)),
    ]
}

/// The box's half size in a frame: the axes swap when the cast runs along x.
fn half_in(frame: Frame) -> Vec2 {
    let h = frame(HALF);
    Vec2::new(h.x.abs(), h.y.abs())
}

/// The velocity in a frame, its zero component being `zero` (0.0 or -0.0).
fn vel_in(frame: Frame, zero: f32) -> Vec2 {
    let v = frame(Vec2::new(zero, 1.0));
    // `frame` may flip the zero's sign; a negative zero is still == 0.0, so restore it
    Vec2::new(
        if v.x == 0.0 { zero } else { v.x },
        if v.y == 0.0 { zero } else { v.y },
    )
}

/// What a cast should report: when the mover first touches the box and when it last does, on
/// an unbounded line through time. `None` if it never touches.
type Expected = Option<(f32, f32)>;

fn close(a: f32, b: f32) -> bool {
    a == b || (a - b).abs() < EPS
}

fn check(name: &str, what: &str, got_toi: Option<f32>, got: Option<(f32, f32)>, exp: Expected) {
    let exp_toi = exp.map(|(enter, _)| enter);
    let ok = match (got, exp) {
        (Some((a, b)), Some((c, d))) => close(a, c) && close(b, d),
        (None, None) => true,
        _ => false,
    };
    let ok_toi = match (got_toi, exp_toi) {
        (Some(a), Some(c)) => close(a, c),
        (None, None) => true,
        _ => false,
    };
    assert!(
        ok && ok_toi,
        "{name}: {what}: got {got_toi:?} / {got:?}, expected {exp_toi:?} / {exp:?}"
    );
}

/// What `toi` and `toiae` make of the unbounded interval: no time travel.
fn forward(exp: Expected) -> Expected {
    exp.and_then(|(enter, exit)| (exit >= 0.0).then_some((enter.max(0.0), exit)))
}

/// Checks `toi` and `toiae`, both ways round (the reverse as the target moving the other way).
macro_rules! check_forward {
    ($name:expr, $mover:expr, $target:expr, $vel:expr, $exp:expr) => {{
        let (mover, target, vel, exp): (_, _, Vec2, Expected) = ($mover, $target, $vel, $exp);
        check(
            $name,
            "toi/toiae",
            mover.toi(&target, vel),
            mover.toiae(&target, vel),
            forward(exp),
        );
        check(
            $name,
            "reversed toi/toiae",
            target.toi(&mover, -vel),
            target.toiae(&mover, -vel),
            forward(exp),
        );
    }};
}

/// Checks every cast, both ways round.
macro_rules! check_all {
    ($name:expr, $mover:expr, $target:expr, $vel:expr, $exp:expr) => {{
        let (mover, target, vel, exp): (_, _, Vec2, Expected) = ($mover, $target, $vel, $exp);
        check_forward!($name, mover, target, vel, exp);
        check(
            $name,
            "tttoi/tttoiae",
            mover.tttoi(&target, vel),
            mover.tttoiae(&target, vel),
            exp,
        );
        check(
            $name,
            "reversed tttoi/tttoiae",
            target.tttoi(&mover, -vel),
            target.tttoiae(&mover, -vel),
            exp,
        );
    }};
}

/// A ball of radius `r` at x = `x` (canonical frame) passing the box, against the expected
/// interval, in every frame and with both kinds of positions.
fn check_ball(x: f32, r: f32, exp: Expected) {
    let ball = Ball::new(r);
    for (frame_name, frame) in frames() {
        let half = half_in(frame);
        let target = Box2d::new(half);
        let shape_target = Shape2d::box2d(half.x, half.y);
        let shape_ball = Shape2d::ball(r);
        let start = frame(Vec2::new(x, START_Y));
        for zero in [0.0, -0.0] {
            let vel = vel_in(frame, zero);
            let name = format!("ball r={r} at x={x}, {frame_name}, vel {vel:?}");
            check_all!(&name, ball.at(&start), target.at(&Vec2::ZERO), vel, exp);
            let (from, to) = (
                Transform2d::from_translation(start),
                Transform2d::from_translation(Vec2::ZERO),
            );
            check_all!(&name, ball.at(&from), target.at(&to), vel, exp);
            // `Shape2d` has only the forward casts
            check_forward!(&name, shape_ball.at(&from), shape_target.at(&to), vel, exp);
            // Both moved off the origin, which changes nothing but the arithmetic
            let shift = Vec2::new(10.0, -20.0);
            let (from, to) = (
                Transform2d::from_translation(start + shift),
                Transform2d::from_translation(shift),
            );
            check_all!(&name, ball.at(&from), target.at(&to), vel, exp);
        }
    }
}

/// Like [`check_ball`] for a point (a ball of radius 0, as a [`Point`]).
fn check_point(x: f32, exp: Expected) {
    for (frame_name, frame) in frames() {
        let half = half_in(frame);
        let target = Box2d::new(half);
        let start = frame(Vec2::new(x, START_Y));
        for zero in [0.0, -0.0] {
            let vel = vel_in(frame, zero);
            let name = format!("point at x={x}, {frame_name}, vel {vel:?}");
            check_all!(&name, Point.at(&start), target.at(&Vec2::ZERO), vel, exp);
            let (from, to) = (
                Transform2d::from_translation(start),
                Transform2d::from_translation(Vec2::ZERO),
            );
            check_all!(&name, Point.at(&from), target.at(&to), vel, exp);
        }
    }
}

/// Like [`check_ball`] for a box of half size `half` (canonical frame).
fn check_box(x: f32, half: Vec2, exp: Expected) {
    for (frame_name, frame) in frames() {
        let target = Box2d::new(half_in(frame));
        let h = frame(half);
        let mover = Box2d::new(Vec2::new(h.x.abs(), h.y.abs()));
        let start = frame(Vec2::new(x, START_Y));
        for zero in [0.0, -0.0] {
            let vel = vel_in(frame, zero);
            let name = format!("box {half:?} at x={x}, {frame_name}, vel {vel:?}");
            check_all!(&name, mover.at(&start), target.at(&Vec2::ZERO), vel, exp);
            let (from, to) = (
                Transform2d::from_translation(start),
                Transform2d::from_translation(Vec2::ZERO),
            );
            check_all!(&name, mover.at(&from), target.at(&to), vel, exp);
        }
    }
}

/// The interval for a mover reaching `reach` above and below the box's center line.
fn passing(reach: f32) -> Expected {
    Some((-START_Y - reach, -START_Y + reach))
}

#[test]
fn ball_inside_the_slab() {
    for x in [0.0, 0.5, -0.5, 0.999] {
        check_ball(x, 0.25, passing(0.75));
    }
}

#[test]
fn ball_centered_on_an_edge_line() {
    // The center runs exactly along the box's side
    for x in [1.0, -1.0] {
        check_ball(x, 0.25, passing(0.75));
    }
}

#[test]
fn ball_touching_a_side_all_along() {
    // Exactly a radius from the side: it touches the whole side on the way past
    for x in [1.25, -1.25] {
        check_ball(x, 0.25, passing(0.5));
    }
}

#[test]
fn ball_past_a_corner() {
    // Between the side's line and a radius beyond it, it meets a rounded corner
    for x in [1.1f32, -1.1, 1.2, -1.2] {
        let dx = x.abs() - HALF.x;
        check_ball(x, 0.25, passing(HALF.y + (0.25f32 * 0.25 - dx * dx).sqrt()));
    }
}

#[test]
fn ball_missing() {
    for x in [1.26, -1.26, 3.0, -3.0] {
        check_ball(x, 0.25, None);
    }
}

#[test]
fn ball_starting_inside_or_beyond() {
    // Starting inside, or past the box: the forward casts must still agree
    for start_offset in [2.5, 3.0, 3.5, 3.75, 4.0, 10.0] {
        let ball = Ball::new(0.25);
        let target = Box2d::new(HALF);
        let start = Vec2::new(1.0, START_Y + start_offset);
        for zero in [0.0, -0.0] {
            let vel = Vec2::new(zero, 1.0);
            let exp = passing(0.75).map(|(a, b)| (a - start_offset, b - start_offset));
            let name = format!("ball from y={}", start.y);
            check_all!(&name, ball.at(&start), target.at(&Vec2::ZERO), vel, exp);
        }
    }
}

#[test]
fn point_along_an_edge_line() {
    for x in [1.0, -1.0] {
        check_point(x, passing(0.5));
    }
    check_point(0.0, passing(0.5));
    check_point(1.01, None);
}

#[test]
fn box_touching_a_side_all_along() {
    // A box whose side slides exactly along the target's side
    let half = Vec2::new(0.25, 0.125);
    for x in [1.25, -1.25] {
        check_box(x, half, passing(0.625));
    }
    check_box(0.0, half, passing(0.625));
    check_box(1.3, half, None);
}

#[test]
fn not_moving() {
    // No velocity at all: touching now is touching forever, apart is never
    let ball = Ball::new(0.25);
    let target = Box2d::new(HALF);
    for zero in [0.0, -0.0] {
        let vel = Vec2::new(zero, zero);
        // Where, and whether a ball of radius 0.25 and a point there touch the box
        for (at, ball_touches, point_touches) in [
            (Vec2::new(0.0, 0.0), true, true),
            (Vec2::new(1.0, 0.5), true, true),
            (Vec2::new(1.25, 0.0), true, false),
            (Vec2::new(1.1, 0.7), true, false),
            (Vec2::new(1.2, 0.7), false, false),
            (Vec2::new(0.0, 3.0), false, false),
        ] {
            let forever = |touches: bool| touches.then_some((f32::NEG_INFINITY, f32::INFINITY));
            let name = format!("still ball at {at:?}, vel {vel:?}");
            let exp = forever(ball_touches);
            check_all!(&name, ball.at(&at), target.at(&Vec2::ZERO), vel, exp);
            let name = format!("still point at {at:?}, vel {vel:?}");
            let exp = forever(point_touches);
            check_all!(&name, Point.at(&at), target.at(&Vec2::ZERO), vel, exp);
        }
    }
}

#[test]
fn a_tiny_sideways_drift_changes_little() {
    // Away from the boundaries, an axis-aligned cast agrees with one drifting a hair sideways
    // (which the general case handles)
    let ball = Ball::new(0.25);
    let target = Box2d::new(HALF);
    for i in -150..=150 {
        let x = i as f32 * 0.01;
        let dx = x.abs() - HALF.x;
        if dx.abs() < 0.01 || (dx - 0.25).abs() < 0.01 {
            continue;
        }
        let start = Vec2::new(x, START_Y);
        let straight = ball.at(&start).toi(&target.at(&Vec2::ZERO), Vec2::Y);
        let drifting = ball
            .at(&start)
            .toi(&target.at(&Vec2::ZERO), Vec2::new(1e-5, 1.0));
        match (straight, drifting) {
            (Some(a), Some(b)) => assert!((a - b).abs() < 1e-3, "x = {x}: {a} vs {b}"),
            (None, None) => {}
            _ => panic!("x = {x}: {straight:?} vs {drifting:?}"),
        }
    }
}

#[test]
fn leaving_a_ball_from_inside() {
    // `toiae` from inside a ball reports when it gets out, not 0
    let ball = Ball::new(0.5);
    let toiae = Point
        .at(&Vec2::new(0.1, 0.0))
        .toiae(&ball.at(&Vec2::ZERO), Vec2::X);
    assert_eq!(toiae, Some((0.0, 0.4)));
    let toiae = Ball::new(0.25)
        .at(&Vec2::new(0.1, 0.0))
        .toiae(&ball.at(&Vec2::ZERO), Vec2::NEG_X);
    let (enter, exit) = toiae.unwrap();
    assert!(enter == 0.0 && close(exit, 0.85), "{toiae:?}");
    // Not moving at all
    let toiae = Point
        .at(&Vec2::new(0.1, 0.0))
        .toiae(&ball.at(&Vec2::ZERO), Vec2::ZERO);
    assert_eq!(toiae, Some((0.0, f32::INFINITY)));
    let toiae = Point
        .at(&Vec2::new(1.0, 0.0))
        .toiae(&ball.at(&Vec2::ZERO), Vec2::ZERO);
    assert_eq!(toiae, None);
}

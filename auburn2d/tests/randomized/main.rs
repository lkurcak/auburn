//! Randomized tests against an exact ground truth.
//!
//! Each test runs seeded cases for a list of shape pairs and compares every query with
//! [`truth`], which is computed in f64 without the library. The cases are the same on every
//! run and platform. `AUBURN_RANDOMIZED_SCALE=20` runs 20 times as many.
//!
//! A pair is in a main test only if it also passes at 20 times the cases. Pairs that fail
//! sit in `#[ignore]`d tests named after the main test plus what is wrong, and the reason
//! names the `BACKLOG.md` details file of every item that has to land first. Some of them
//! only fail at the larger scale, so check an item with
//! `AUBURN_RANDOMIZED_SCALE=20 cargo test --release --all-features -- --ignored`, then move
//! the pairs that pass into the main test.

mod check;
mod sample;
mod truth;

use auburn2d::prelude::*;
use auburn2d::transformation::pos2d::Pos2d;
use check::{Hit, Kind, Report, cases};
use sample::Config;

const STATIC_CASES: usize = 1000;
const CAST_CASES: usize = 300;

/// `static_test!(name: check @ T, config, [A-B, ...])` runs `check::check` on each pair,
/// with both shapes placed by a `T`.
macro_rules! static_test {
    ($(#[$attr:meta])* $name:ident: $check:ident @ $T:ty, $cfg:expr, [$($A:ident-$B:ident),* $(,)?]) => {
        $(#[$attr])*
        #[test]
        fn $name() {
            let mut report = Report::default();
            $( report.push(check::$check::<$A, $B, $T>(&$cfg, cases(STATIC_CASES))); )*
            report.finish();
        }
    };
}

macro_rules! query {
    (Toi) => {
        |a, b, v| Hit::Time(a.toi(b, v))
    };
    (TtToi) => {
        |a, b, v| Hit::Time(a.tttoi(b, v))
    };
    (Toiae) => {
        |a, b, v| Hit::Span(a.toiae(b, v))
    };
    (TtToiae) => {
        |a, b, v| Hit::Span(a.tttoiae(b, v))
    };
}

/// `cast_test!(name: Kind @ T, config, [A-B, ...])` checks the cast `Kind` on each pair, with
/// `a` moving. `Kind at rest` checks it at zero velocity instead.
macro_rules! cast_test {
    ($(#[$attr:meta])* $name:ident: $kind:ident @ $T:ty, $cfg:expr, [$($A:ident-$B:ident),* $(,)?]) => {
        $(#[$attr])*
        #[test]
        fn $name() {
            let mut report = Report::default();
            $( report.push(check::cast::<$A, $B, $T>(&$cfg, cases(CAST_CASES), Kind::$kind, query!($kind))); )*
            report.finish();
        }
    };
    ($(#[$attr:meta])* $name:ident: $kind:ident at rest @ $T:ty, $cfg:expr, [$($A:ident-$B:ident),* $(,)?]) => {
        $(#[$attr])*
        #[test]
        fn $name() {
            let mut report = Report::default();
            $( report.push(check::cast_at_rest::<$A, $B, $T>(&$cfg, cases(CAST_CASES), Kind::$kind, query!($kind))); )*
            report.finish();
        }
    };
}

mod collides {
    use super::*;
    static_test!(
        vec2: collides @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        pos2d: collides @ Pos2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        scale_translate2d: collides @ ScaleTranslate2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        scale_translate2d_mirrored: collides @ ScaleTranslate2d, Config::MIRRORED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d: collides @ Transform2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d_mirrored: collides @ Transform2d, Config::MIRRORED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d_tiny: collides @ Transform2d, Config::TINY,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d_huge: collides @ Transform2d, Config::HUGE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        vec2_far: collides @ Vec2, Config::FAR,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        #[cfg(feature = "bevy")]
        bevy_transform: collides @ bevy::prelude::Transform, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball,
            Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
}

mod penetrates {
    use super::*;
    static_test!(
        vec2: penetrates @ Vec2, Config::BASE,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        pos2d: penetrates @ Pos2d, Config::BASE,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        scale_translate2d: penetrates @ ScaleTranslate2d, Config::BASE,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        scale_translate2d_mirrored: penetrates @ ScaleTranslate2d, Config::MIRRORED,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d: penetrates @ Transform2d, Config::BASE,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d_mirrored: penetrates @ Transform2d, Config::MIRRORED,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d_tiny: penetrates @ Transform2d, Config::TINY,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        transform2d_huge: penetrates @ Transform2d, Config::HUGE,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        vec2_far: penetrates @ Vec2, Config::FAR,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
    static_test!(
        #[cfg(feature = "bevy")]
        bevy_transform: penetrates @ bevy::prelude::Transform, Config::BASE,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
}

mod toi {
    use super::*;
    cast_test!(
        vec2: Toi @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_shape_cast: Toi @ Vec2, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        vec2_tiny: Toi @ Vec2, Config::TINY,
        [
            Point-Point, Point-Ball, Point-Box2d, Ball-Point, Ball-Ball, Ball-Box2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Shape2d-Point, Shape2d-Ball,
            Shape2d-Box2d, Point-Shape2d, Ball-Shape2d, Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_tiny_shape_cast: Toi @ Vec2, Config::TINY,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        #[ignore = "backlog/poly2d-robustness.md"]
        vec2_tiny_poly2d: Toi @ Vec2, Config::TINY,
        [Point-Poly2d, Poly2d-Box2d, Poly2d-Poly2d]
    );
    cast_test!(
        vec2_huge: Toi @ Vec2, Config::HUGE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_huge_shape_cast: Toi @ Vec2, Config::HUGE,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d_axis_aligned: Toi @ Transform2d, Config::AXIS_ALIGNED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_axis_aligned_shape_cast: Toi @ Transform2d, Config::AXIS_ALIGNED,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d_rigid: Toi @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_rigid_shape_cast: Toi @ Transform2d, Config::RIGID,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d: Toi @ Transform2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_shape_cast: Toi @ Transform2d, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d_mirrored: Toi @ Transform2d, Config::MIRRORED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_mirrored_shape_cast: Toi @ Transform2d, Config::MIRRORED,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        vec2_at_rest: Toi at rest @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point,
            Poly2d-Ball, Poly2d-Box2d, Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d,
            Shape2d-Poly2d, Point-Shape2d, Ball-Shape2d, Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        transform2d_at_rest: Toi at rest @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point,
            Poly2d-Ball, Poly2d-Box2d, Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d,
            Shape2d-Poly2d, Point-Shape2d, Ball-Shape2d, Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
}

mod toiae {
    use super::*;
    cast_test!(
        vec2: Toiae @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_shape_cast: Toiae @ Vec2, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        vec2_tiny: Toiae @ Vec2, Config::TINY,
        [
            Point-Point, Point-Ball, Point-Box2d, Ball-Point, Ball-Ball, Ball-Box2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Box2d, Shape2d-Point, Shape2d-Ball,
            Shape2d-Box2d, Point-Shape2d, Ball-Shape2d, Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_tiny_shape_cast: Toiae @ Vec2, Config::TINY,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        #[ignore = "backlog/poly2d-robustness.md"]
        vec2_tiny_poly2d: Toiae @ Vec2, Config::TINY,
        [Point-Poly2d, Poly2d-Point, Poly2d-Poly2d]
    );
    cast_test!(
        vec2_huge: Toiae @ Vec2, Config::HUGE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_huge_shape_cast: Toiae @ Vec2, Config::HUGE,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d_axis_aligned: Toiae @ Transform2d, Config::AXIS_ALIGNED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_axis_aligned_shape_cast: Toiae @ Transform2d, Config::AXIS_ALIGNED,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d_rigid: Toiae @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_rigid_shape_cast: Toiae @ Transform2d, Config::RIGID,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d: Toiae @ Transform2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_shape_cast: Toiae @ Transform2d, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        transform2d_mirrored: Toiae @ Transform2d, Config::MIRRORED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d, Point-Shape2d, Ball-Shape2d,
            Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_mirrored_shape_cast: Toiae @ Transform2d, Config::MIRRORED,
        [Ball-Poly2d, Poly2d-Ball, Shape2d-Poly2d]
    );
    cast_test!(
        vec2_at_rest: Toiae at rest @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point,
            Poly2d-Ball, Poly2d-Box2d, Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d,
            Shape2d-Poly2d, Point-Shape2d, Ball-Shape2d, Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
    cast_test!(
        transform2d_at_rest: Toiae at rest @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point,
            Poly2d-Ball, Poly2d-Box2d, Poly2d-Poly2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d,
            Shape2d-Poly2d, Point-Shape2d, Ball-Shape2d, Box2d-Shape2d, Shape2d-Shape2d
        ]
    );
}

mod tttoi {
    use super::*;
    cast_test!(
        vec2: TtToi @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_shape_cast: TtToi @ Vec2, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        vec2_tiny: TtToi @ Vec2, Config::TINY,
        [
            Point-Point, Point-Ball, Point-Box2d, Ball-Point, Ball-Ball, Ball-Box2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Box2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_tiny_shape_cast: TtToi @ Vec2, Config::TINY,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        #[ignore = "backlog/poly2d-robustness.md"]
        vec2_tiny_poly2d: TtToi @ Vec2, Config::TINY,
        [Point-Poly2d, Poly2d-Point, Poly2d-Poly2d]
    );
    cast_test!(
        vec2_huge: TtToi @ Vec2, Config::HUGE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_huge_shape_cast: TtToi @ Vec2, Config::HUGE,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d_axis_aligned: TtToi @ Transform2d, Config::AXIS_ALIGNED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_axis_aligned_shape_cast: TtToi @ Transform2d, Config::AXIS_ALIGNED,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d_rigid: TtToi @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_rigid_shape_cast: TtToi @ Transform2d, Config::RIGID,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d: TtToi @ Transform2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_shape_cast: TtToi @ Transform2d, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d_mirrored: TtToi @ Transform2d, Config::MIRRORED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_mirrored_shape_cast: TtToi @ Transform2d, Config::MIRRORED,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        vec2_at_rest: TtToi at rest @ Vec2, Config::BASE,
        [
            Point-Point, Point-Box2d, Point-Poly2d, Ball-Box2d, Ball-Poly2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Ball, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec2_at_rest_edge_cases: TtToi at rest @ Vec2, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        transform2d_at_rest: TtToi at rest @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Box2d, Point-Poly2d, Ball-Box2d, Ball-Poly2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Ball, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        transform2d_at_rest_edge_cases: TtToi at rest @ Transform2d, Config::RIGID,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
}

mod tttoiae {
    use super::*;
    cast_test!(
        vec2: TtToiae @ Vec2, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_shape_cast: TtToiae @ Vec2, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        vec2_tiny: TtToiae @ Vec2, Config::TINY,
        [
            Point-Point, Point-Ball, Point-Box2d, Ball-Point, Ball-Ball, Ball-Box2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Box2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_tiny_shape_cast: TtToiae @ Vec2, Config::TINY,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        #[ignore = "backlog/poly2d-robustness.md"]
        vec2_tiny_poly2d: TtToiae @ Vec2, Config::TINY,
        [Point-Poly2d, Poly2d-Point, Poly2d-Poly2d]
    );
    cast_test!(
        vec2_huge: TtToiae @ Vec2, Config::HUGE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec2_huge_shape_cast: TtToiae @ Vec2, Config::HUGE,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d_axis_aligned: TtToiae @ Transform2d, Config::AXIS_ALIGNED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_axis_aligned_shape_cast: TtToiae @ Transform2d, Config::AXIS_ALIGNED,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d_rigid: TtToiae @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_rigid_shape_cast: TtToiae @ Transform2d, Config::RIGID,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d: TtToiae @ Transform2d, Config::BASE,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_shape_cast: TtToiae @ Transform2d, Config::BASE,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        transform2d_mirrored: TtToiae @ Transform2d, Config::MIRRORED,
        [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Ball-Point, Ball-Ball, Ball-Box2d,
            Box2d-Point, Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform2d_mirrored_shape_cast: TtToiae @ Transform2d, Config::MIRRORED,
        [Ball-Poly2d, Poly2d-Ball]
    );
    cast_test!(
        vec2_at_rest: TtToiae at rest @ Vec2, Config::BASE,
        [
            Point-Point, Point-Box2d, Point-Poly2d, Ball-Box2d, Ball-Poly2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Ball, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec2_at_rest_edge_cases: TtToiae at rest @ Vec2, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        transform2d_at_rest: TtToiae at rest @ Transform2d, Config::RIGID,
        [
            Point-Point, Point-Box2d, Point-Poly2d, Ball-Box2d, Ball-Poly2d, Box2d-Point,
            Box2d-Ball, Box2d-Box2d, Box2d-Poly2d, Poly2d-Point, Poly2d-Ball, Poly2d-Box2d,
            Poly2d-Poly2d
        ]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        transform2d_at_rest_edge_cases: TtToiae at rest @ Transform2d, Config::RIGID,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
}

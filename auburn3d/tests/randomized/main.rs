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

use auburn3d::prelude::*;
use auburn3d::transformation::pos3d::Pos3d;
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
        vec3: collides @ Vec3, Config::BASE,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        pos3d: collides @ Pos3d, Config::BASE,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        scale_translate3d: collides @ ScaleTranslate3d, Config::BASE,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        scale_translate3d_mirrored: collides @ ScaleTranslate3d, Config::MIRRORED,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d: collides @ Transform3d, Config::BASE,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d_mirrored: collides @ Transform3d, Config::MIRRORED,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d_tiny: collides @ Transform3d, Config::TINY,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d_huge: collides @ Transform3d, Config::HUGE,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        vec3_far: collides @ Vec3, Config::FAR,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        #[cfg(feature = "bevy")]
        bevy_transform: collides @ bevy::prelude::Transform, Config::BASE,
        [Point-Point, Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
}

mod penetrates {
    use super::*;
    static_test!(
        vec3: penetrates @ Vec3, Config::BASE,
        [Point-Ball, Point-Box3d, Ball-Ball, Box3d-Box3d]
    );
    static_test!(
        #[ignore = "backlog/gjk-3d-cycling.md"]
        vec3_gjk: penetrates @ Vec3, Config::BASE,
        [Ball-Box3d]
    );
    static_test!(
        pos3d: penetrates @ Pos3d, Config::BASE,
        [Point-Ball, Point-Box3d, Ball-Ball, Box3d-Box3d]
    );
    static_test!(
        #[ignore = "backlog/gjk-3d-cycling.md"]
        pos3d_gjk: penetrates @ Pos3d, Config::BASE,
        [Ball-Box3d]
    );
    static_test!(
        scale_translate3d: penetrates @ ScaleTranslate3d, Config::BASE,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        scale_translate3d_mirrored: penetrates @ ScaleTranslate3d, Config::MIRRORED,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d: penetrates @ Transform3d, Config::BASE,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d_mirrored: penetrates @ Transform3d, Config::MIRRORED,
        [Point-Ball, Point-Box3d, Ball-Ball, Box3d-Box3d]
    );
    static_test!(
        #[ignore = "backlog/gjk-3d-cycling.md"]
        transform3d_mirrored_gjk: penetrates @ Transform3d, Config::MIRRORED,
        [Ball-Box3d]
    );
    static_test!(
        transform3d_tiny: penetrates @ Transform3d, Config::TINY,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        transform3d_huge: penetrates @ Transform3d, Config::HUGE,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        vec3_far: penetrates @ Vec3, Config::FAR,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        #[cfg(feature = "bevy")]
        bevy_transform: penetrates @ bevy::prelude::Transform, Config::BASE,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
}

mod toi {
    use super::*;
    cast_test!(
        vec3: Toi @ Vec3, Config::BASE,
        [Point-Point, Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_shape_cast: Toi @ Vec3, Config::BASE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_tiny: Toi @ Vec3, Config::TINY,
        [Point-Point, Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_tiny_shape_cast: Toi @ Vec3, Config::TINY,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_huge: Toi @ Vec3, Config::HUGE,
        [Point-Point, Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_huge_shape_cast: Toi @ Vec3, Config::HUGE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        transform3d_axis_aligned: Toi @ Transform3d, Config::AXIS_ALIGNED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_axis_aligned_shape_cast: Toi @ Transform3d, Config::AXIS_ALIGNED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_rigid: Toi @ Transform3d, Config::RIGID,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_rigid_shape_cast: Toi @ Transform3d, Config::RIGID,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d: Toi @ Transform3d, Config::BASE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_shape_cast: Toi @ Transform3d, Config::BASE,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_mirrored: Toi @ Transform3d, Config::MIRRORED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_mirrored_shape_cast: Toi @ Transform3d, Config::MIRRORED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        vec3_at_rest: Toi at rest @ Vec3, Config::BASE,
        [Point-Point, Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_at_rest_edge_cases: Toi at rest @ Vec3, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        transform3d_at_rest: Toi at rest @ Transform3d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point,
            Box3d-Ball, Box3d-Box3d
        ]
    );
}

mod toiae {
    use super::*;
    cast_test!(
        vec3: Toiae @ Vec3, Config::BASE,
        [Point-Point, Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_shape_cast: Toiae @ Vec3, Config::BASE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_tiny: Toiae @ Vec3, Config::TINY,
        [Point-Point, Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_tiny_shape_cast: Toiae @ Vec3, Config::TINY,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_huge: Toiae @ Vec3, Config::HUGE,
        [Point-Point, Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_huge_shape_cast: Toiae @ Vec3, Config::HUGE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        transform3d_axis_aligned: Toiae @ Transform3d, Config::AXIS_ALIGNED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_axis_aligned_shape_cast: Toiae @ Transform3d, Config::AXIS_ALIGNED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_rigid: Toiae @ Transform3d, Config::RIGID,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_rigid_shape_cast: Toiae @ Transform3d, Config::RIGID,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d: Toiae @ Transform3d, Config::BASE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_shape_cast: Toiae @ Transform3d, Config::BASE,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_mirrored: Toiae @ Transform3d, Config::MIRRORED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_mirrored_shape_cast: Toiae @ Transform3d, Config::MIRRORED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        vec3_at_rest: Toiae at rest @ Vec3, Config::BASE,
        [Point-Point, Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_at_rest_edge_cases: Toiae at rest @ Vec3, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        transform3d_at_rest: Toiae at rest @ Transform3d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point,
            Box3d-Ball, Box3d-Box3d
        ]
    );
}

mod tttoi {
    use super::*;
    cast_test!(
        vec3: TtToi @ Vec3, Config::BASE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_edge_cases: TtToi @ Vec3, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_shape_cast: TtToi @ Vec3, Config::BASE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_tiny: TtToi @ Vec3, Config::TINY,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_tiny_edge_cases: TtToi @ Vec3, Config::TINY,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_tiny_shape_cast: TtToi @ Vec3, Config::TINY,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_huge: TtToi @ Vec3, Config::HUGE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_huge_edge_cases: TtToi @ Vec3, Config::HUGE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_huge_shape_cast: TtToi @ Vec3, Config::HUGE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        transform3d_axis_aligned: TtToi @ Transform3d, Config::AXIS_ALIGNED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_axis_aligned_shape_cast: TtToi @ Transform3d, Config::AXIS_ALIGNED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_rigid: TtToi @ Transform3d, Config::RIGID,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_rigid_shape_cast: TtToi @ Transform3d, Config::RIGID,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d: TtToi @ Transform3d, Config::BASE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_shape_cast: TtToi @ Transform3d, Config::BASE,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_mirrored: TtToi @ Transform3d, Config::MIRRORED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_mirrored_shape_cast: TtToi @ Transform3d, Config::MIRRORED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        vec3_at_rest: TtToi at rest @ Vec3, Config::BASE,
        [Point-Point, Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_at_rest_edge_cases: TtToi at rest @ Vec3, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        transform3d_at_rest: TtToi at rest @ Transform3d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point,
            Box3d-Ball, Box3d-Box3d
        ]
    );
}

mod tttoiae {
    use super::*;
    cast_test!(
        vec3: TtToiae @ Vec3, Config::BASE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_edge_cases: TtToiae @ Vec3, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_shape_cast: TtToiae @ Vec3, Config::BASE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_tiny: TtToiae @ Vec3, Config::TINY,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_tiny_edge_cases: TtToiae @ Vec3, Config::TINY,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_tiny_shape_cast: TtToiae @ Vec3, Config::TINY,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        vec3_huge: TtToiae @ Vec3, Config::HUGE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_huge_edge_cases: TtToiae @ Vec3, Config::HUGE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        vec3_huge_shape_cast: TtToiae @ Vec3, Config::HUGE,
        [Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        transform3d_axis_aligned: TtToiae @ Transform3d, Config::AXIS_ALIGNED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_axis_aligned_shape_cast: TtToiae @ Transform3d, Config::AXIS_ALIGNED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_rigid: TtToiae @ Transform3d, Config::RIGID,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_rigid_shape_cast: TtToiae @ Transform3d, Config::RIGID,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d: TtToiae @ Transform3d, Config::BASE,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_shape_cast: TtToiae @ Transform3d, Config::BASE,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        transform3d_mirrored: TtToiae @ Transform3d, Config::MIRRORED,
        [Point-Point]
    );
    cast_test!(
        #[ignore = "backlog/generic-shape-cast.md"]
        transform3d_mirrored_shape_cast: TtToiae @ Transform3d, Config::MIRRORED,
        [
            Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point, Box3d-Ball,
            Box3d-Box3d
        ]
    );
    cast_test!(
        vec3_at_rest: TtToiae at rest @ Vec3, Config::BASE,
        [Point-Point, Point-Box3d, Ball-Box3d, Box3d-Point, Box3d-Ball, Box3d-Box3d]
    );
    cast_test!(
        #[ignore = "backlog/cast-edge-cases.md"]
        vec3_at_rest_edge_cases: TtToiae at rest @ Vec3, Config::BASE,
        [Point-Ball, Ball-Point, Ball-Ball]
    );
    cast_test!(
        transform3d_at_rest: TtToiae at rest @ Transform3d, Config::RIGID,
        [
            Point-Point, Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point,
            Box3d-Ball, Box3d-Box3d
        ]
    );
}

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
/// `a` moving and both shapes placed by a `T`; `@ (TA, TB)` places `a` by a `TA` and `b` by a
/// `TB`. `Kind at rest` checks it at zero velocity instead.
macro_rules! cast_test {
    ($(#[$attr:meta])* $name:ident: $kind:ident @ ($TA:ty, $TB:ty), $cfg:expr, [$($A:ident-$B:ident),* $(,)?]) => {
        $(#[$attr])*
        #[test]
        fn $name() {
            let mut report = Report::default();
            $( report.push(check::cast::<$A, $B, $TA, $TB>(&$cfg, cases(CAST_CASES), Kind::$kind, query!($kind))); )*
            report.finish();
        }
    };
    ($(#[$attr:meta])* $name:ident: $kind:ident at rest @ ($TA:ty, $TB:ty), $cfg:expr, [$($A:ident-$B:ident),* $(,)?]) => {
        $(#[$attr])*
        #[test]
        fn $name() {
            let mut report = Report::default();
            $( report.push(check::cast_at_rest::<$A, $B, $TA, $TB>(&$cfg, cases(CAST_CASES), Kind::$kind, query!($kind))); )*
            report.finish();
        }
    };
    ($(#[$attr:meta])* $name:ident: $kind:ident @ $T:ty, $cfg:expr, $pairs:tt) => {
        cast_test!($(#[$attr])* $name: $kind @ ($T, $T), $cfg, $pairs);
    };
    ($(#[$attr:meta])* $name:ident: $kind:ident at rest @ $T:ty, $cfg:expr, $pairs:tt) => {
        cast_test!($(#[$attr])* $name: $kind at rest @ ($T, $T), $cfg, $pairs);
    };
}

/// `every_pair!(test!(args,))` is `test!(args, [A-B, ...])` with every ordered pair of the
/// shapes the checks sample.
macro_rules! every_pair {
    ($test:ident!($($args:tt)*)) => {
        $test!($($args)* [
            Point-Point, Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Point,
            Ball-Ball, Ball-Box2d, Ball-Poly2d, Ball-Shape2d, Box2d-Point, Box2d-Ball,
            Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Point, Poly2d-Ball, Poly2d-Box2d,
            Poly2d-Poly2d, Poly2d-Shape2d, Shape2d-Point, Shape2d-Ball, Shape2d-Box2d,
            Shape2d-Poly2d, Shape2d-Shape2d
        ]);
    };
}

/// The tests of the cast `Kind`: every pair, under every transformation and some mixes of them.
macro_rules! cast_tests {
    ($kind:ident) => {
        every_pair!(cast_test!(vec2: $kind @ Vec2, Config::BASE,));
        every_pair!(cast_test!(vec2_tiny: $kind @ Vec2, Config::TINY,));
        every_pair!(cast_test!(vec2_huge: $kind @ Vec2, Config::HUGE,));
        every_pair!(cast_test!(vec2_far: $kind @ Vec2, Config::FAR,));
        every_pair!(cast_test!(pos2d: $kind @ Pos2d, Config::BASE,));
        every_pair!(cast_test!(scale_translate2d: $kind @ ScaleTranslate2d, Config::BASE,));
        every_pair!(cast_test!(
            scale_translate2d_mirrored: $kind @ ScaleTranslate2d, Config::MIRRORED,
        ));
        every_pair!(cast_test!(
            transform2d_axis_aligned: $kind @ Transform2d, Config::AXIS_ALIGNED,
        ));
        every_pair!(cast_test!(transform2d_rigid: $kind @ Transform2d, Config::RIGID,));
        every_pair!(cast_test!(transform2d: $kind @ Transform2d, Config::BASE,));
        every_pair!(cast_test!(transform2d_mirrored: $kind @ Transform2d, Config::MIRRORED,));
        every_pair!(cast_test!(transform2d_tiny: $kind @ Transform2d, Config::TINY,));
        every_pair!(cast_test!(transform2d_huge: $kind @ Transform2d, Config::HUGE,));
        every_pair!(cast_test!(transform2d_far: $kind @ Transform2d, Config::FAR,));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform: $kind @ bevy::prelude::Transform, Config::BASE,
        ));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform_mirrored: $kind @ bevy::prelude::Transform, Config::MIRRORED,
        ));
        every_pair!(cast_test!(vec2_and_transform2d: $kind @ (Vec2, Transform2d), Config::BASE,));
        every_pair!(cast_test!(
            transform2d_and_scale_translate2d: $kind @ (Transform2d, ScaleTranslate2d),
            Config::MIRRORED,
        ));
        every_pair!(cast_test!(
            scale_translate2d_and_pos2d: $kind @ (ScaleTranslate2d, Pos2d), Config::MIRRORED,
        ));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform_and_transform2d: $kind @ (bevy::prelude::Transform, Transform2d),
            Config::MIRRORED,
        ));
        every_pair!(cast_test!(vec2_at_rest: $kind at rest @ Vec2, Config::BASE,));
        every_pair!(cast_test!(transform2d_at_rest: $kind at rest @ Transform2d, Config::RIGID,));
        every_pair!(cast_test!(
            vec2_and_transform2d_at_rest: $kind at rest @ (Vec2, Transform2d), Config::BASE,
        ));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform_at_rest: $kind at rest @ bevy::prelude::Transform, Config::MIRRORED,
        ));
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
    static_test!(
        #[cfg(feature = "bevy")]
        bevy_transform_mirrored: collides @ bevy::prelude::Transform, Config::MIRRORED,
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
    static_test!(
        #[cfg(feature = "bevy")]
        bevy_transform_mirrored: penetrates @ bevy::prelude::Transform, Config::MIRRORED,
        [
            Point-Ball, Point-Box2d, Point-Poly2d, Point-Shape2d, Ball-Ball, Ball-Box2d,
            Ball-Poly2d, Ball-Shape2d, Box2d-Box2d, Box2d-Poly2d, Box2d-Shape2d, Poly2d-Poly2d,
            Poly2d-Shape2d, Shape2d-Shape2d
        ]
    );
}

mod toi {
    use super::*;
    cast_tests!(Toi);
}

mod toiae {
    use super::*;
    cast_tests!(Toiae);
}

mod tttoi {
    use super::*;
    cast_tests!(TtToi);
}

mod tttoiae {
    use super::*;
    cast_tests!(TtToiae);
}

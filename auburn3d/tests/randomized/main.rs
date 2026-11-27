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
            Point-Point, Point-Ball, Point-Box3d, Ball-Point, Ball-Ball, Ball-Box3d, Box3d-Point,
            Box3d-Ball, Box3d-Box3d
        ]);
    };
}

/// The tests of the cast `Kind`: every pair, under every transformation and some mixes of them.
macro_rules! cast_tests {
    ($kind:ident) => {
        every_pair!(cast_test!(vec3: $kind @ Vec3, Config::BASE,));
        every_pair!(cast_test!(vec3_tiny: $kind @ Vec3, Config::TINY,));
        every_pair!(cast_test!(vec3_huge: $kind @ Vec3, Config::HUGE,));
        every_pair!(cast_test!(vec3_far: $kind @ Vec3, Config::FAR,));
        every_pair!(cast_test!(pos3d: $kind @ Pos3d, Config::BASE,));
        every_pair!(cast_test!(scale_translate3d: $kind @ ScaleTranslate3d, Config::BASE,));
        every_pair!(cast_test!(
            scale_translate3d_mirrored: $kind @ ScaleTranslate3d, Config::MIRRORED,
        ));
        every_pair!(cast_test!(
            transform3d_axis_aligned: $kind @ Transform3d, Config::AXIS_ALIGNED,
        ));
        every_pair!(cast_test!(transform3d_rigid: $kind @ Transform3d, Config::RIGID,));
        every_pair!(cast_test!(transform3d: $kind @ Transform3d, Config::BASE,));
        every_pair!(cast_test!(transform3d_mirrored: $kind @ Transform3d, Config::MIRRORED,));
        every_pair!(cast_test!(transform3d_tiny: $kind @ Transform3d, Config::TINY,));
        every_pair!(cast_test!(transform3d_huge: $kind @ Transform3d, Config::HUGE,));
        every_pair!(cast_test!(transform3d_far: $kind @ Transform3d, Config::FAR,));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform: $kind @ bevy::prelude::Transform, Config::BASE,
        ));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform_mirrored: $kind @ bevy::prelude::Transform, Config::MIRRORED,
        ));
        every_pair!(cast_test!(vec3_and_transform3d: $kind @ (Vec3, Transform3d), Config::BASE,));
        every_pair!(cast_test!(
            transform3d_and_scale_translate3d: $kind @ (Transform3d, ScaleTranslate3d),
            Config::MIRRORED,
        ));
        every_pair!(cast_test!(
            scale_translate3d_and_pos3d: $kind @ (ScaleTranslate3d, Pos3d), Config::MIRRORED,
        ));
        every_pair!(cast_test!(
            #[cfg(feature = "bevy")]
            bevy_transform_and_transform3d: $kind @ (bevy::prelude::Transform, Transform3d),
            Config::MIRRORED,
        ));
        every_pair!(cast_test!(vec3_at_rest: $kind at rest @ Vec3, Config::BASE,));
        every_pair!(cast_test!(transform3d_at_rest: $kind at rest @ Transform3d, Config::RIGID,));
        every_pair!(cast_test!(
            vec3_and_transform3d_at_rest: $kind at rest @ (Vec3, Transform3d), Config::BASE,
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
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
    );
    static_test!(
        pos3d: penetrates @ Pos3d, Config::BASE,
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
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
        [Point-Ball, Point-Box3d, Ball-Ball, Ball-Box3d, Box3d-Box3d]
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

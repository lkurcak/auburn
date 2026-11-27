use auburn2d::prelude::*;
use std::fmt::Debug;

fn test_isometry2d_invariants<I: Transformation2d + Debug + PartialEq>(iso: &I) {
    assert_eq!(iso.apply_to_origin(), iso.apply(Vec2::ZERO));
    assert_eq!(iso.inverse().apply(iso.apply_to_origin()), Vec2::ZERO);
    assert_eq!(&iso.inverse().inverse(), iso);

    let sample_points = [
        Vec2::ZERO,
        Vec2::X,
        Vec2::Y,
        Vec2::ONE,
        Vec2::NEG_X,
        Vec2::NEG_Y,
        Vec2::new(1.0, 2.0),
        Vec2::new(-1.0, -2.0),
        Vec2::new(1.0, -2.0),
        Vec2::new(-1.0, 2.0),
        Vec2::new(0.5, 0.5),
        Vec2::new(-0.5, -0.5),
        Vec2::new(0.5, -0.5),
        Vec2::new(-0.5, 0.5),
    ];

    for &point in &sample_points {
        assert_eq!(iso.inverse().apply(iso.apply(point)), point);
        assert_eq!(iso.apply(iso.inverse().apply(point)), point);
    }
    test_support_direction(iso);
}

/// `support_direction` is the transpose of the linear part: `support_direction(d) · p` equals
/// `d · (apply(p) - apply_to_origin())`.
fn test_support_direction<I: Transformation2d + Debug>(iso: &I) {
    let samples = [
        Vec2::X,
        Vec2::NEG_Y,
        Vec2::new(1.0, 2.0),
        Vec2::new(-0.5, 0.25),
        Vec2::new(3.0, -4.0),
    ];
    for d in samples {
        for p in samples {
            let got = iso.support_direction(d).dot(p);
            let want = d.dot(iso.apply(p) - iso.apply_to_origin());
            let tolerance = 1e-5 * (1.0 + want.abs());
            assert!(
                (got - want).abs() < tolerance,
                "{iso:?}, {d}, {p}: {got} != {want}"
            );
        }
    }
}

#[test]
fn vec2() {
    let isometries = [
        Vec2::ZERO,
        Vec2::X,
        Vec2::Y,
        Vec2::ONE,
        Vec2::NEG_X,
        Vec2::NEG_Y,
    ];

    for iso in &isometries {
        test_isometry2d_invariants(iso);
    }
}

#[test]
fn scale_translate2d() {
    let isometries = [
        ScaleTranslate2d::IDENTITY,
        ScaleTranslate2d::from_scale(1.0),
        ScaleTranslate2d::from_scale(2.0),
        ScaleTranslate2d::from_translation(Vec2::ONE),
        ScaleTranslate2d::from_translation(Vec2::new(2.0, 3.0)),
        ScaleTranslate2d::new(Vec2::ONE, 1.0),
        ScaleTranslate2d::new(Vec2::new(2.0, 3.0), 4.0),
    ];

    for iso in &isometries {
        test_isometry2d_invariants(iso);
    }
}

#[test]
fn transform2d_support_direction() {
    let isometries = [
        Transform2d::IDENTITY,
        Transform2d::from_translation(Vec2::new(2.0, 3.0)),
        Transform2d::from_angle(0.5),
        Transform2d::from_scale(2.0),
        Transform2d::from_scale(-0.5),
        Transform2d::from_translation(Vec2::new(5.0, -7.0))
            .with_angle(-2.0)
            .with_scale(3.0),
    ];

    for iso in &isometries {
        test_support_direction(iso);
    }
}

#[test]
fn pos2d_support_direction() {
    use auburn2d::transformation::pos2d::Pos2d;
    test_support_direction(&Pos2d::new(5.0, -7.0));
}

#[cfg(feature = "bevy")]
#[test]
fn bevy_transform_support_direction() {
    use bevy::prelude::{Quat, Transform, Vec3};
    let isometries = [
        Transform::IDENTITY,
        Transform::from_xyz(5.0, -7.0, 3.0),
        Transform::from_rotation(Quat::from_rotation_z(0.5)).with_scale(Vec3::splat(2.0)),
    ];

    for iso in &isometries {
        test_support_direction(iso);
    }
}

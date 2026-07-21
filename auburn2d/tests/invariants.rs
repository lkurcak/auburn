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

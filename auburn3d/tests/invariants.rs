use auburn3d::prelude::*;
use auburn3d::Quat;
use std::fmt::Debug;

fn test_isometry3d_invariants<I: Transformation3d + Debug + PartialEq>(iso: &I) {
    assert_eq!(iso.apply_to_origin(), iso.apply(Vec3::ZERO));
    assert_eq!(iso.inverse().apply(iso.apply_to_origin()), Vec3::ZERO);
    assert_eq!(&iso.inverse().inverse(), iso);

    let sample_points = [
        Vec3::ZERO,
        Vec3::X,
        Vec3::Y,
        Vec3::Z,
        Vec3::ONE,
        Vec3::NEG_X,
        Vec3::NEG_Y,
        Vec3::NEG_Z,
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(-1.0, -2.0, -3.0),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(-0.5, -0.5, -0.5),
    ];

    for &point in &sample_points {
        let diff = (iso.inverse().apply(iso.apply(point)) - point).length();
        assert!(diff < 1e-5, "inverse(apply(point)) != point");
        let diff = (iso.apply(iso.inverse().apply(point)) - point).length();
        assert!(diff < 1e-5, "apply(inverse(point)) != point");
    }
}

#[test]
fn vec3() {
    let isometries = [
        Vec3::ZERO,
        Vec3::X,
        Vec3::Y,
        Vec3::Z,
        Vec3::ONE,
        Vec3::NEG_X,
        Vec3::NEG_Y,
        Vec3::NEG_Z,
    ];

    for iso in &isometries {
        test_isometry3d_invariants(iso);
    }
}

#[test]
fn scale_translate3d() {
    let isometries = [
        ScaleTranslate3d::IDENTITY,
        ScaleTranslate3d::from_scale(1.0),
        ScaleTranslate3d::from_scale(2.0),
        ScaleTranslate3d::from_translation(Vec3::ONE),
        ScaleTranslate3d::from_translation(Vec3::new(2.0, 3.0, 4.0)),
        ScaleTranslate3d::new(Vec3::ONE, 1.0),
        ScaleTranslate3d::new(Vec3::new(2.0, 3.0, 4.0), 4.0),
    ];

    for iso in &isometries {
        test_isometry3d_invariants(iso);
    }
}

#[test]
fn transform3d() {
    let isometries = [
        Transform3d::IDENTITY,
        Transform3d::from_translation(Vec3::ONE),
        Transform3d::from_translation(Vec3::new(2.0, 3.0, 4.0)),
        Transform3d::from_rotation(Quat::from_rotation_x(0.5)),
        Transform3d::from_rotation(Quat::from_rotation_y(0.5)),
        Transform3d::from_rotation(Quat::from_rotation_z(0.5)),
        Transform3d::from_scale(2.0),
    ];

    for iso in &isometries {
        test_isometry3d_invariants(iso);
    }
}

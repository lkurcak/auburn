use auburn3d::Quat;
use auburn3d::prelude::*;
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
    test_support_direction(iso);
}

/// `support_direction` is the transpose of the linear part: `support_direction(d) · p` equals
/// `d · (apply(p) - apply_to_origin())`.
fn test_support_direction<I: Transformation3d + Debug>(iso: &I) {
    let samples = [
        Vec3::X,
        Vec3::NEG_Y,
        Vec3::Z,
        Vec3::new(1.0, 2.0, 3.0),
        Vec3::new(-0.5, 0.25, -2.0),
        Vec3::new(3.0, -4.0, 0.5),
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

#[test]
fn transform3d_support_direction() {
    let axis = Vec3::new(1.0, 2.0, 3.0).normalize();
    let isometries = [
        Transform3d::from_scale(-0.5),
        Transform3d::from_rotation(Quat::from_axis_angle(axis, 0.7))
            .with_translation(Vec3::new(5.0, -7.0, 2.0))
            .with_scale(3.0),
    ];

    for iso in &isometries {
        test_support_direction(iso);
    }
}

#[test]
fn pos3d_support_direction() {
    use auburn3d::transformation::pos3d::Pos3d;
    test_support_direction(&Pos3d(Vec3::new(5.0, -7.0, 2.0)));
}

/// Unlike the other types, bevy's `Transform` can scale each axis differently.
#[cfg(feature = "bevy")]
#[test]
fn bevy_transform_support_direction() {
    use bevy::prelude::Transform;
    let axis = Vec3::new(1.0, 2.0, 3.0).normalize();
    let isometries = [
        Transform::IDENTITY,
        Transform::from_xyz(5.0, -7.0, 2.0),
        Transform::from_rotation(Quat::from_axis_angle(axis, 0.7))
            .with_scale(Vec3::new(1.0, 4.0, 0.5)),
        Transform::from_scale(Vec3::new(-1.0, 1.0, 1.0)),
    ];

    for iso in &isometries {
        test_support_direction(iso);
    }
}

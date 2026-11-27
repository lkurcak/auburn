//! Casts between the built-in shapes, worked out by hand: the pairs that had no casts before
//! every pair went through one generic cast.

use auburn3d::prelude::*;
use auburn3d::transformation::pos3d::Pos3d;

fn near(got: Option<(f32, f32)>, want: (f32, f32)) -> bool {
    got.is_some_and(|(a, b)| (a - want.0).abs() < 1e-4 && (b - want.1).abs() < 1e-4)
}

#[test]
fn pos3d() {
    let (ball, cube) = (Ball::new(1.0), Box3d::cube(1.0));
    let (a, b) = (
        Pos3d(Vec3::new(-5.0, 0.0, 0.0)),
        Pos3d(Vec3::new(0.0, 0.5, 0.5)),
    );
    let got = ball.at(&a).toiae(&cube.at(&b), Vec3::new(2.0, 0.0, 0.0));
    assert!(near(got, (1.5, 3.5)), "{got:?}");
}

#[test]
fn scale_translate3d() {
    // The cube is scaled to a half size of 2, mirrored, and the ball to a radius of 0.5.
    let (ball, cube) = (Ball::new(1.0), Box3d::cube(1.0));
    let a = ScaleTranslate3d::new(Vec3::new(-5.0, 0.0, 0.0), 0.5);
    let b = ScaleTranslate3d::new(Vec3::ZERO, -2.0);
    let got = ball.at(&a).tttoiae(&cube.at(&b), Vec3::X);
    assert!(near(got, (2.5, 7.5)), "{got:?}");
}

#[test]
fn mixed_transformations() {
    // A point placed by a `Vec3` against a cube turned 45° about z by a `Transform3d`: its
    // edge is at x = -√2.
    let cube = Box3d::cube(1.0);
    let a = Vec3::new(-5.0, 0.0, 0.0);
    let b =
        Transform3d::from_rotation(auburn3d::Quat::from_rotation_z(std::f32::consts::FRAC_PI_4));
    let s = std::f32::consts::SQRT_2;
    let got = Point.at(&a).toiae(&cube.at(&b), Vec3::X);
    assert!(near(got, (5.0 - s, 5.0 + s)), "{got:?}");
    let got = cube.at(&b).toiae(&Point.at(&a), Vec3::NEG_X);
    assert!(near(got, (5.0 - s, 5.0 + s)), "{got:?}");
}

#[cfg(feature = "bevy")]
#[test]
fn bevy_transform_stretches_a_ball() {
    // A unit ball stretched to an ellipsoid with half axes 2, 1 and 1.
    use bevy::prelude::Transform;
    let ball = Ball::new(1.0);
    let tr = Transform::from_scale(Vec3::new(2.0, 1.0, 1.0));
    let got = Point
        .at(&Vec3::new(-5.0, 0.0, 0.0))
        .toiae(&ball.at(&tr), Vec3::X);
    assert!(near(got, (3.0, 7.0)), "{got:?}");
    let got = Point
        .at(&Vec3::new(0.0, -5.0, 0.0))
        .toiae(&ball.at(&tr), Vec3::Y);
    assert!(near(got, (4.0, 6.0)), "{got:?}");
}

/// A shape of the user's own, with only a support function.
struct Octahedron;

impl Shape for Octahedron {
    fn volume(&self) -> f32 {
        4.0 / 3.0
    }
}

impl Support for Octahedron {
    fn support(&self, dir: Vec3) -> Vec3 {
        let a = dir.abs();
        if a.x >= a.y && a.x >= a.z {
            Vec3::new(dir.x.signum(), 0.0, 0.0)
        } else if a.y >= a.z {
            Vec3::new(0.0, dir.y.signum(), 0.0)
        } else {
            Vec3::new(0.0, 0.0, dir.z.signum())
        }
    }
}

#[test]
fn a_shape_with_only_a_support_function() {
    let (a, b) = (Vec3::new(-5.0, 0.0, 0.0), Vec3::ZERO);
    let got = Octahedron.at(&a).toiae(&Box3d::cube(1.0).at(&b), Vec3::X);
    assert!(near(got, (3.0, 7.0)), "{got:?}");
    let got = Ball::new(1.0)
        .at(&b)
        .tttoiae(&Octahedron.at(&a), Vec3::NEG_X);
    assert!(near(got, (3.0, 7.0)), "{got:?}");
}

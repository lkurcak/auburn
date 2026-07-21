use auburn2d::{prelude::*, shape::shape2d::Shape2d};

#[test]
fn shape2d_ball_ball_collides() {
    let ball = Shape2d::ball(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let ball2 = Shape2d::ball(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    assert!(a.collides(&b));
}

#[test]
fn shape2d_vs_ball_penetrates() {
    let ball = Shape2d::ball(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    assert!(a.collides(&b));

    // GJK/EPA may have slight numerical differences from analytical solutions
    let penetration = a.penetrates(&b).expect("Should penetrate");
    let expected = Vec2::new(-0.121320374, -0.121320374);
    let diff = (penetration - expected).length();
    assert!(
        diff < 0.001,
        "Penetration {penetration:?} differs from expected {expected:?} by {diff}"
    );

    assert!(b.collides(&a));

    let penetration_reverse = b.penetrates(&a).expect("Should penetrate");
    let expected_reverse = Vec2::new(0.121320374, 0.121320374);
    let diff_reverse = (penetration_reverse - expected_reverse).length();
    assert!(
        diff_reverse < 0.001,
        "Penetration {penetration_reverse:?} differs from expected {expected_reverse:?} by {diff_reverse}"
    );
}

#[test]
fn shape2d_vs_box() {
    let ball = Shape2d::ball(1.0);
    let pos = Vec2::new(1.125, 2.0);
    let a = ball.at(&pos);

    let ball2 = Box2d::new(Vec2::new(1.0, 2.0));
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    // GJK/EPA may have slight numerical differences
    let penetration = a.penetrates(&b).expect("Should penetrate");
    let expected = Vec2::new(-0.125, 0.0);
    let diff = (penetration - expected).length();
    assert!(
        diff < 0.01,
        "Penetration {penetration:?} differs from expected {expected:?} by {diff}"
    );
}

#[test]
fn shape2d_ball_ball_penetrates() {
    let ball = Shape2d::ball(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let ball2 = Shape2d::ball(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    // GJK/EPA may have slight numerical differences
    let penetration = a.penetrates(&b).expect("Should penetrate");
    let expected = Vec2::new(-0.121320374, -0.121320374);
    let diff = (penetration - expected).length();
    assert!(
        diff < 0.001,
        "Penetration {penetration:?} differs from expected {expected:?} by {diff}"
    );
}

#[test]
fn shape2d_ball_ball_toi() {
    let ball = Shape2d::ball(0.75);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let rect = Shape2d::box2d(1.0, 2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = rect.at(&pos2);

    assert_eq!(a.toi(&b, Vec2::X), Some(0.25));
    assert_eq!(a.toi(&b, Vec2::Y), None);
}

#[test]
fn shape2d_support() {
    let ball = Shape2d::ball(2.0);
    assert_eq!(ball.support(Vec2::X), 2.0 * Vec2::X);

    let rect = Shape2d::box2d(1.0, 2.0);
    assert_eq!(rect.support(Vec2::new(1.0, -1.0)), Vec2::new(1.0, -2.0));
}

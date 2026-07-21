use auburn2d::prelude::*;

#[test]
fn ball_ball_collides_1() {
    let ball = Ball::new(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    assert!(a.collides(&b));
}

#[test]
fn ball_ball_collides_2() {
    let ball = Ball::new(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    let penetration = a.penetrates(&b).expect("Should penetrate");

    // GJK/EPA may have slight numerical differences from analytical solutions
    // Check that the penetration vector is approximately correct
    let expected = Vec2::new(-0.121320374, -0.121320374);
    let diff = (penetration - expected).length();
    assert!(
        diff < 0.001,
        "Penetration vector {penetration:?} differs from expected {expected:?} by {diff}"
    );
}

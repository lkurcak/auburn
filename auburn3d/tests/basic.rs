use auburn3d::prelude::*;

#[test]
fn ball_ball_collides_1() {
    let ball = Ball::new(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec3::new(2.0, 3.0, 4.0);
    let b = ball2.at(&pos2);

    assert!(a.collides(&b));
}

#[test]
fn ball_ball_collides_2() {
    let ball = Ball::new(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec3::new(2.0, 3.0, 4.0);
    let b = ball2.at(&pos2);

    let penetration = a.penetrates(&b).expect("Should penetrate");
    assert!(penetration.length() > 0.0);
}

#[test]
fn ball_ball_not_collides() {
    let ball = Ball::new(1.0);
    let pos = Vec3::new(0.0, 0.0, 0.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(1.0);
    let pos2 = Vec3::new(5.0, 0.0, 0.0);
    let b = ball2.at(&pos2);

    assert!(!a.collides(&b));
    assert_eq!(a.penetrates(&b), None);
}

#[test]
fn box_box_collides() {
    let box1 = Box3d::cube(1.0);
    let pos1 = Vec3::new(0.0, 0.0, 0.0);
    let a = box1.at(&pos1);

    let box2 = Box3d::cube(1.0);
    let pos2 = Vec3::new(1.0, 0.0, 0.0);
    let b = box2.at(&pos2);

    assert!(a.collides(&b));
    let penetration = a.penetrates(&b).expect("Should penetrate");
    assert!(penetration.length() > 0.0);
}

#[test]
fn ball_box_collides() {
    let ball = Ball::new(1.0);
    let pos1 = Vec3::new(0.0, 0.0, 0.0);
    let a = ball.at(&pos1);

    let box1 = Box3d::cube(1.0);
    let pos2 = Vec3::new(1.5, 0.0, 0.0);
    let b = box1.at(&pos2);

    assert!(a.collides(&b));
}

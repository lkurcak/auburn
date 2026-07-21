use auburn3d::Vec3;
use auburn3d::shape::ball::Ball;
use auburn3d::shape::box3d::Box3d;
use auburn3d::shape::point::Point;

#[test]
fn inherent_at_method() {
    let ball = Ball::new(1.0);
    let pos = Vec3::ZERO;
    let _collider = ball.at(&pos);

    let box3d = Box3d::cube(1.0);
    let _collider = box3d.at(&pos);

    let point = Point;
    let _collider = point.at(&pos);
}

#[test]
fn inherent_relation_methods() {
    let ball1 = Ball::new(1.0);
    let pos1 = Vec3::ZERO;
    let c1 = ball1.at(&pos1);

    let ball2 = Ball::new(1.0);
    let pos2 = Vec3::new(0.5, 0.0, 0.0);
    let c2 = ball2.at(&pos2);

    assert!(c1.collides(&c2));

    let pen = c1.penetrates(&c2);
    assert!(pen.is_some());

    let ball3 = Ball::new(1.0);
    let pos3 = Vec3::new(10.0, 0.0, 0.0);
    let c3 = ball3.at(&pos3);

    let vel = Vec3::new(-1.0, 0.0, 0.0);
    let toi = c3.toi(&c1, vel);
    assert!(toi.is_some());
    assert!(toi.unwrap() > 0.0);
}

use auburn2d::Vec2;
use auburn2d::shape::ball::Ball;
use auburn2d::shape::box2d::Box2d;
use auburn2d::shape::point::Point;
use auburn2d::shape::shape2d::Shape2d;

// NOTE: We are intentionally NOT importing any traits here.
// No `use auburn2d::prelude::*;`
// No `use auburn2d::shape::Shape;`
// No `use auburn2d::relation::collides::Collides;`

#[test]
fn inherent_at_method() {
    let ball = Ball::new(1.0);
    let pos = Vec2::ZERO;
    // This should compile if `Ball` has an inherent `at` method
    let _collider = ball.at(&pos);

    let box2d = Box2d::square(1.0);
    // This should compile if `Box2d` has an inherent `at` method
    let _collider = box2d.at(&pos);

    let point = Point;
    // This should compile if `Point` has an inherent `at` method
    let _collider = point.at(&pos);

    let shape2d = Shape2d::ball(1.0);
    // This should compile if `Shape2d` has an inherent `at` method
    let _collider = shape2d.at(&pos);
}

#[test]
fn inherent_relation_methods() {
    let ball1 = Ball::new(1.0);
    let pos1 = Vec2::ZERO;
    let c1 = ball1.at(&pos1);

    let ball2 = Ball::new(1.0);
    let pos2 = Vec2::new(0.5, 0.0); // overlapping
    let c2 = ball2.at(&pos2);

    // Test `collides` inherent method
    assert!(c1.collides(&c2));

    // Test `penetrates` inherent method
    let pen = c1.penetrates(&c2);
    assert!(pen.is_some());

    // Test `toi` inherent method
    // Move ball2 far away and cast towards ball1
    let ball3 = Ball::new(1.0);
    let pos3 = Vec2::new(10.0, 0.0);
    let c3 = ball3.at(&pos3);

    let vel = Vec2::new(-1.0, 0.0);
    let toi = c3.toi(&c1, vel);
    assert!(toi.is_some());
    assert!(toi.unwrap() > 0.0);
}

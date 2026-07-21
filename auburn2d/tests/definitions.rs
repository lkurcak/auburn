use auburn2d::prelude::*;

// Point

#[test]
fn point_point_definitions() {
    let point = Point;
    let pos = Vec2::new(1.0, 2.0);
    let a = point.at(&pos);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = point.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec2::X));
    println!("{:?}", a.toiae(&b, Vec2::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec2::X));
    println!("{:?}", b.toiae(&a, Vec2::X));
}

#[test]
fn ball_point_definitions() {
    let ball = Ball::new(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let point = Point;
    let pos2 = Vec2::new(3.0, 4.0);
    let b = point.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec2::X));
    println!("{:?}", a.toiae(&b, Vec2::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec2::X));
    println!("{:?}", b.toiae(&a, Vec2::X));
}

#[test]
fn box_point_definitions() {
    let box1 = Box2d::square(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = box1.at(&pos);

    let point = Point;
    let pos2 = Vec2::new(3.0, 4.0);
    let b = point.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec2::X));
    println!("{:?}", a.toiae(&b, Vec2::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec2::X));
    println!("{:?}", b.toiae(&a, Vec2::X));
}

// Ball

#[test]
fn ball_ball_definitions() {
    let ball = Ball::new(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = ball2.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec2::X));
    println!("{:?}", a.toiae(&b, Vec2::X));
}

#[test]
fn ball_box_definitions() {
    let ball = Ball::new(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = ball.at(&pos);

    let box2 = Box2d::square(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = box2.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec2::X));
    println!("{:?}", a.toiae(&b, Vec2::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec2::X));
    println!("{:?}", b.toiae(&a, Vec2::X));
}

// Box2d

#[test]
fn box_box_definitions() {
    let box1 = Box2d::square(1.0);
    let pos = Vec2::new(1.0, 2.0);
    let a = box1.at(&pos);

    let box2 = Box2d::square(2.0);
    let pos2 = Vec2::new(3.0, 4.0);
    let b = box2.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec2::X));
    println!("{:?}", a.toiae(&b, Vec2::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec2::X));
    println!("{:?}", b.toiae(&a, Vec2::X));
}

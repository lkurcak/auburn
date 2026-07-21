use auburn3d::prelude::*;

#[test]
fn point_point_definitions() {
    let point = Point;
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = point.at(&pos);
    let pos2 = Vec3::new(3.0, 4.0, 5.0);
    let b = point.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec3::X));
    println!("{:?}", a.toiae(&b, Vec3::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec3::X));
    println!("{:?}", b.toiae(&a, Vec3::X));
}

#[test]
fn ball_point_definitions() {
    let ball = Ball::new(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = ball.at(&pos);

    let point = Point;
    let pos2 = Vec3::new(3.0, 4.0, 5.0);
    let b = point.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec3::X));
    println!("{:?}", a.toiae(&b, Vec3::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec3::X));
    println!("{:?}", b.toiae(&a, Vec3::X));
}

#[test]
fn box_point_definitions() {
    let box1 = Box3d::cube(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = box1.at(&pos);

    let point = Point;
    let pos2 = Vec3::new(3.0, 4.0, 5.0);
    let b = point.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec3::X));
    println!("{:?}", a.toiae(&b, Vec3::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec3::X));
    println!("{:?}", b.toiae(&a, Vec3::X));
}

#[test]
fn ball_ball_definitions() {
    let ball = Ball::new(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = ball.at(&pos);

    let ball2 = Ball::new(2.0);
    let pos2 = Vec3::new(3.0, 4.0, 5.0);
    let b = ball2.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec3::X));
    println!("{:?}", a.toiae(&b, Vec3::X));
}

#[test]
fn ball_box_definitions() {
    let ball = Ball::new(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = ball.at(&pos);

    let box2 = Box3d::cube(2.0);
    let pos2 = Vec3::new(3.0, 4.0, 5.0);
    let b = box2.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec3::X));
    println!("{:?}", a.toiae(&b, Vec3::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec3::X));
    println!("{:?}", b.toiae(&a, Vec3::X));
}

#[test]
fn box_box_definitions() {
    let box1 = Box3d::cube(1.0);
    let pos = Vec3::new(1.0, 2.0, 3.0);
    let a = box1.at(&pos);

    let box2 = Box3d::cube(2.0);
    let pos2 = Vec3::new(3.0, 4.0, 5.0);
    let b = box2.at(&pos2);

    println!("{:?}", a.collides(&b));
    println!("{:?}", a.penetrates(&b));
    println!("{:?}", a.toi(&b, Vec3::X));
    println!("{:?}", a.toiae(&b, Vec3::X));

    println!("{:?}", b.collides(&a));
    println!("{:?}", b.penetrates(&a));
    println!("{:?}", b.toi(&a, Vec3::X));
    println!("{:?}", b.toiae(&a, Vec3::X));
}

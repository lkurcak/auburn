//! Every relation for every pair of shapes at `Vec2`, with the answers worked out by hand.
//!
//! The first shape sits at (1, 2) and the second at (3, 4). Casts move the first along +x
//! (`a.toi(&b, X)`), or the second (`b.toi(&a, X)`).

use auburn2d::prelude::*;

fn assert_time(got: Option<f32>, want: Option<f32>) {
    match (got, want) {
        (Some(g), Some(w)) => assert!((g - w).abs() < 1e-4, "got {g}, want {w}"),
        _ => assert_eq!(got, want),
    }
}

fn assert_span(got: Option<(f32, f32)>, want: Option<(f32, f32)>) {
    match (got, want) {
        (Some(g), Some(w)) => assert!(
            (g.0 - w.0).abs() < 1e-4 && (g.1 - w.1).abs() < 1e-4,
            "got {g:?}, want {w:?}"
        ),
        _ => assert_eq!(got, want),
    }
}

/// `p` has length `depth` and points into the quadrant of `toward`'s signs.
fn assert_push(p: Option<Vec2>, depth: f32, toward: Vec2) {
    let p = p.expect("the shapes overlap");
    assert!(
        (p.length() - depth).abs() < 1e-3,
        "{p:?} should have length {depth}"
    );
    assert!(
        p.x * toward.x >= 0.0 && p.y * toward.y >= 0.0,
        "{p:?} should point along {toward:?}"
    );
}

/// Asserts that the shapes are apart and stay apart when either moves along +x.
fn assert_apart<A, B>(a: &A, b: &B)
where
    A: Collides<B> + Penetrates<B> + TimeOfImpact<B> + TimeOfImpactAndExit<B>,
    B: Collides<A> + Penetrates<A> + TimeOfImpact<A> + TimeOfImpactAndExit<A>,
{
    assert!(!a.collides(b) && !b.collides(a));
    assert_eq!(a.penetrates(b), None);
    assert_eq!(b.penetrates(a), None);
    assert_eq!(a.toi(b, Vec2::X), None);
    assert_eq!(a.toiae(b, Vec2::X), None);
    assert_eq!(b.toi(a, Vec2::X), None);
    assert_eq!(b.toiae(a, Vec2::X), None);
}

const A: Vec2 = Vec2::new(1.0, 2.0);
const B: Vec2 = Vec2::new(3.0, 4.0);

#[test]
fn point_point_definitions() {
    assert_apart(&Point.at(&A), &Point.at(&B));
}

#[test]
fn ball_point_definitions() {
    // 2.83 apart, and 2 apart across the line of motion: more than the radius.
    assert_apart(&Ball::new(1.0).at(&A), &Point.at(&B));
}

#[test]
fn box_point_definitions() {
    // The box covers y in [1, 3]; the point is at y = 4.
    assert_apart(&Box2d::square(1.0).at(&A), &Point.at(&B));
}

#[test]
fn ball_ball_definitions() {
    let (ball1, ball2) = (Ball::new(1.0), Ball::new(2.0));
    let (a, b) = (ball1.at(&A), ball2.at(&B));
    // The centres are 2√2 apart, so the balls overlap by 3 - 2√2.
    let depth = 3.0 - 2.0 * 2f32.sqrt();
    assert!(a.collides(&b) && b.collides(&a));
    let p = a.penetrates(&b).unwrap();
    assert!(
        (p - Vec2::splat(-depth / 2f32.sqrt())).length() < 1e-3,
        "{p:?}"
    );
    let q = b.penetrates(&a).unwrap();
    assert!(
        (q - Vec2::splat(depth / 2f32.sqrt())).length() < 1e-3,
        "{q:?}"
    );
    assert_time(a.toi(&b, Vec2::X), Some(0.0));
    // They part when (t - 2)² + 4 = 9 for a moving, and (t + 2)² + 4 = 9 for b moving.
    assert_span(a.toiae(&b, Vec2::X), Some((0.0, 2.0 + 5f32.sqrt())));
    assert_time(b.toi(&a, Vec2::X), Some(0.0));
    assert_span(b.toiae(&a, Vec2::X), Some((0.0, 5f32.sqrt() - 2.0)));
}

#[test]
fn ball_box_definitions() {
    let (ball, square) = (Ball::new(1.0), Box2d::square(2.0));
    let (a, b) = (ball.at(&A), square.at(&B));
    // The ball's centre is on the box's corner (1, 2), so it is a radius deep, and any push
    // out of the corner's quadrant works.
    assert!(a.collides(&b) && b.collides(&a));
    assert_push(a.penetrates(&b), 1.0, Vec2::NEG_ONE);
    assert_push(b.penetrates(&a), 1.0, Vec2::ONE);
    assert_time(a.toi(&b, Vec2::X), Some(0.0));
    // The centre runs along the box's bottom edge, x in [1, 5], plus the radius.
    assert_span(a.toiae(&b, Vec2::X), Some((0.0, 5.0)));
    assert_time(b.toi(&a, Vec2::X), Some(0.0));
    assert_span(b.toiae(&a, Vec2::X), Some((0.0, 1.0)));
}

#[test]
fn box_box_definitions() {
    let (small, large) = (Box2d::square(1.0), Box2d::square(2.0));
    let (a, b) = (small.at(&A), large.at(&B));
    // [0, 2] x [1, 3] against [1, 5] x [2, 6]: one unit deep along both axes.
    assert!(a.collides(&b) && b.collides(&a));
    let p = a.penetrates(&b).unwrap();
    assert!(
        [Vec2::NEG_X, Vec2::NEG_Y]
            .iter()
            .any(|w| (p - *w).length() < 1e-4),
        "{p:?}"
    );
    let q = b.penetrates(&a).unwrap();
    assert!(
        [Vec2::X, Vec2::Y].iter().any(|w| (q - *w).length() < 1e-4),
        "{q:?}"
    );
    assert_time(a.toi(&b, Vec2::X), Some(0.0));
    assert_span(a.toiae(&b, Vec2::X), Some((0.0, 5.0)));
    assert_time(b.toi(&a, Vec2::X), Some(0.0));
    assert_span(b.toiae(&a, Vec2::X), Some((0.0, 1.0)));
}

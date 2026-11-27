//! Every relation for every pair of shapes at `Vec3`, with the answers worked out by hand.
//!
//! The first shape sits at (1, 2, 3) and the second at (3, 4, 5). Casts move the first along
//! +x (`a.toi(&b, X)`), or the second (`b.toi(&a, X)`).

use auburn3d::prelude::*;

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

/// `p` has length `depth` and points into the octant of `toward`'s signs.
fn assert_push(p: Option<Vec3>, depth: f32, toward: Vec3) {
    let p = p.expect("the shapes overlap");
    assert!(
        (p.length() - depth).abs() < 1e-3,
        "{p:?} should have length {depth}"
    );
    assert!(
        (p * toward).cmpge(Vec3::ZERO).all(),
        "{p:?} should point along {toward:?}"
    );
}

/// `p` is one of the unit pushes along `-x`, `-y` or `-z`, times `sign`.
fn assert_axis_push(p: Option<Vec3>, sign: f32) {
    let p = p.expect("the shapes overlap");
    let ok = [Vec3::X, Vec3::Y, Vec3::Z]
        .iter()
        .any(|&axis| (p - axis * sign).length() < 1e-4);
    assert!(ok, "{p:?} should be a unit push along an axis, sign {sign}");
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
    assert_eq!(a.toi(b, Vec3::X), None);
    assert_eq!(a.toiae(b, Vec3::X), None);
    assert_eq!(b.toi(a, Vec3::X), None);
    assert_eq!(b.toiae(a, Vec3::X), None);
}

const A: Vec3 = Vec3::new(1.0, 2.0, 3.0);
const B: Vec3 = Vec3::new(3.0, 4.0, 5.0);

#[test]
fn point_point_definitions() {
    assert_apart(&Point.at(&A), &Point.at(&B));
}

#[test]
fn ball_point_definitions() {
    // 2√3 apart, and 2√2 apart across the line of motion: more than the radius.
    assert_apart(&Ball::new(1.0).at(&A), &Point.at(&B));
}

#[test]
fn box_point_definitions() {
    // The box covers y in [1, 3]; the point is at y = 4.
    assert_apart(&Box3d::cube(1.0).at(&A), &Point.at(&B));
}

#[test]
fn ball_ball_definitions() {
    let (ball1, ball2) = (Ball::new(1.0), Ball::new(2.0));
    let (a, b) = (ball1.at(&A), ball2.at(&B));
    // The centres are 2√3 apart, more than the radii's sum of 3.
    assert!(!a.collides(&b) && !b.collides(&a));
    assert_eq!(a.penetrates(&b), None);
    // The balls touch while (t - 2)² + 8 <= 9 for a moving, and (t + 2)² + 8 <= 9 for b: in
    // the past.
    assert_time(a.toi(&b, Vec3::X), Some(1.0));
    assert_time(b.toi(&a, Vec3::X), None);
    assert_span(b.toiae(&a, Vec3::X), None);
}

#[test]
fn ball_ball_toiae() {
    let (ball1, ball2) = (Ball::new(1.0), Ball::new(2.0));
    assert_span(ball1.at(&A).toiae(&ball2.at(&B), Vec3::X), Some((1.0, 3.0)));
}

#[test]
fn ball_box_definitions() {
    let (ball, cube) = (Ball::new(1.0), Box3d::cube(2.0));
    let (a, b) = (ball.at(&A), cube.at(&B));
    // The ball's centre is on the box's corner (1, 2, 3).
    assert!(a.collides(&b) && b.collides(&a));
    assert_time(a.toi(&b, Vec3::X), Some(0.0));
    assert_time(b.toi(&a, Vec3::X), Some(0.0));
}

#[test]
#[ignore = "backlog/epa-3d.md"]
fn ball_box_penetrates() {
    let (ball, cube) = (Ball::new(1.0), Box3d::cube(2.0));
    let (a, b) = (ball.at(&A), cube.at(&B));
    // A radius deep, and any push out of the corner's octant works.
    assert_push(a.penetrates(&b), 1.0, Vec3::NEG_ONE);
    assert_push(b.penetrates(&a), 1.0, Vec3::ONE);
}

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn ball_box_toiae() {
    let (ball, cube) = (Ball::new(1.0), Box3d::cube(2.0));
    let (a, b) = (ball.at(&A), cube.at(&B));
    // The centre runs along the box's edge, x in [1, 5], plus the radius.
    assert_span(a.toiae(&b, Vec3::X), Some((0.0, 5.0)));
    assert_span(b.toiae(&a, Vec3::X), Some((0.0, 1.0)));
}

#[test]
fn box_box_definitions() {
    let (small, large) = (Box3d::cube(1.0), Box3d::cube(2.0));
    let (a, b) = (small.at(&A), large.at(&B));
    // [0, 2] x [1, 3] x [2, 4] against [1, 5] x [2, 6] x [3, 7]: a unit deep along every axis.
    assert!(a.collides(&b) && b.collides(&a));
    assert_time(a.toi(&b, Vec3::X), Some(0.0));
    assert_time(b.toi(&a, Vec3::X), Some(0.0));
    assert_span(b.toiae(&a, Vec3::X), Some((0.0, 1.0)));
}

#[test]
#[ignore = "backlog/epa-3d.md"]
fn box_box_penetrates() {
    let (small, large) = (Box3d::cube(1.0), Box3d::cube(2.0));
    let (a, b) = (small.at(&A), large.at(&B));
    assert_axis_push(a.penetrates(&b), -1.0);
    assert_axis_push(b.penetrates(&a), 1.0);
}

#[test]
#[ignore = "backlog/generic-shape-cast.md"]
fn box_box_toiae() {
    let (small, large) = (Box3d::cube(1.0), Box3d::cube(2.0));
    assert_span(small.at(&A).toiae(&large.at(&B), Vec3::X), Some((0.0, 5.0)));
}

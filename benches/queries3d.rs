//! auburn3d's basic queries, next to parry3d's and the formulas a game would write by hand.
//!
//! As in 2D, every benchmark answers one query for each of [`PAIRS`] pairs of shapes in turn,
//! the same pairs on every run. Its id says which:
//! `3d/<query>/<moving shape>-<other shape>/<transformation>/<implementation>`, the
//! transformation being a `Vec3` position or a turned `Transform3d`. 3D has no `cast` yet.

mod common;
mod formula3d;

use std::f32::consts::TAU;

use auburn3d::prelude::*;
use auburn3d::{Quat, Vec3};
use criterion::{Criterion, criterion_group, criterion_main};

use common::{PAIRS, Rng, compare};
use formula3d::Body;

/// A pair: `a` moves at the velocity, `b` stands still.
struct Pair {
    a: Body,
    b: Body,
    vel: Vec3,
}

/// The same pairs on every run, turned or not.
fn pairs(turned: bool) -> Vec<Pair> {
    let mut rng = Rng::new(2);
    (0..PAIRS)
        .map(|_| {
            let b = body(&mut rng, Vec3::ZERO, 10.0, turned);
            let a = body(&mut rng, b.pos, 3.0, turned);
            let vel = vector(&mut rng, 8.0);
            Pair { a, b, vel }
        })
        .collect()
}

/// A shape up to `spread` from `around` along each axis, of every size at once: its shape
/// takes the radius or the half size it needs.
fn body(rng: &mut Rng, around: Vec3, spread: f32, turned: bool) -> Body {
    let pos = around + vector(rng, spread);
    let rot = if turned {
        let axis = vector(rng, 1.0).normalize_or(Vec3::X);
        Quat::from_axis_angle(axis, rng.range(0.0, TAU))
    } else {
        Quat::IDENTITY
    };
    Body {
        pos,
        rot,
        radius: rng.range(0.5, 1.5),
        half_size: Vec3::new(
            rng.range(0.5, 1.5),
            rng.range(0.5, 1.5),
            rng.range(0.5, 1.5),
        ),
    }
}

/// Up to `size` along each axis.
fn vector(rng: &mut Rng, size: f32) -> Vec3 {
    Vec3::new(
        rng.range(-size, size),
        rng.range(-size, size),
        rng.range(-size, size),
    )
}

/// The pairs as auburn's shapes and transformations.
mod auburn {
    use super::{Body, Pair, Queries};
    use auburn3d::Vec3;
    use auburn3d::prelude::*;

    pub trait Make {
        fn make(body: &Body) -> Self;
    }

    impl Make for Point {
        fn make(_: &Body) -> Self {
            Point
        }
    }

    impl Make for Ball {
        fn make(body: &Body) -> Self {
            Ball::new(body.radius)
        }
    }

    impl Make for Box3d {
        fn make(body: &Body) -> Self {
            Box3d::new(body.half_size)
        }
    }

    pub trait Place {
        fn place(body: &Body) -> Self;
    }

    impl Place for Vec3 {
        fn place(body: &Body) -> Self {
            body.pos
        }
    }

    impl Place for Transform3d {
        fn place(body: &Body) -> Self {
            Transform3d {
                pos: body.pos,
                rot: body.rot,
                scale: 1.0,
            }
        }
    }

    pub struct Of<A, B, T> {
        a: A,
        at_a: T,
        b: B,
        at_b: T,
        vel: Vec3,
    }

    pub fn of<A: Make, B: Make, T: Place>(pairs: &[Pair]) -> Vec<Of<A, B, T>> {
        pairs
            .iter()
            .map(|pair| Of {
                a: A::make(&pair.a),
                at_a: T::place(&pair.a),
                b: B::make(&pair.b),
                at_b: T::place(&pair.b),
                vel: pair.vel,
            })
            .collect()
    }

    impl<A, B, T> Queries for Of<A, B, T>
    where
        A: Shape + Support,
        B: Shape + Support,
        T: Transformation3d,
    {
        fn collides(&self) -> bool {
            self.a.at(&self.at_a).collides(&self.b.at(&self.at_b))
        }

        fn penetrates(&self) -> Option<Vec3> {
            self.a.at(&self.at_a).penetrates(&self.b.at(&self.at_b))
        }

        fn toi(&self) -> Option<f32> {
            self.a.at(&self.at_a).toi(&self.b.at(&self.at_b), self.vel)
        }
    }
}

/// The pairs as parry's shapes and poses, with its answers in auburn's terms. A point is a
/// point query or a ray cast, as parry has no point shape.
mod parry {
    use super::{Body, Pair, Queries};
    use auburn3d::Vec3;
    use parry3d::math::{Pose, Rotation, Vector};
    use parry3d::query::{self, Ray, ShapeCastOptions};
    use parry3d::shape::{Ball, Cuboid, Shape};

    fn to_parry(v: Vec3) -> Vector {
        Vector::new(v.x, v.y, v.z)
    }

    fn from_parry(v: Vector) -> Vec3 {
        Vec3::new(v.x, v.y, v.z)
    }

    fn pose(body: &Body) -> Pose {
        let [x, y, z, w] = body.rot.to_array();
        Pose::from_parts(to_parry(body.pos), Rotation::from_xyzw(x, y, z, w))
    }

    pub trait Make {
        fn make(body: &Body) -> Self;
    }

    impl Make for Ball {
        fn make(body: &Body) -> Self {
            Ball::new(body.radius)
        }
    }

    impl Make for Cuboid {
        fn make(body: &Body) -> Self {
            Cuboid::new(to_parry(body.half_size))
        }
    }

    pub struct Of<A, B> {
        a: A,
        pose_a: Pose,
        b: B,
        pose_b: Pose,
        vel: Vector,
    }

    pub fn of<A: Make, B: Make>(pairs: &[Pair]) -> Vec<Of<A, B>> {
        pairs
            .iter()
            .map(|pair| Of {
                a: A::make(&pair.a),
                pose_a: pose(&pair.a),
                b: B::make(&pair.b),
                pose_b: pose(&pair.b),
                vel: to_parry(pair.vel),
            })
            .collect()
    }

    impl<A: Shape, B: Shape> Of<A, B> {
        fn shape_cast(&self, options: ShapeCastOptions) -> Option<query::ShapeCastHit> {
            query::cast_shapes(
                &self.pose_a,
                self.vel,
                &self.a,
                &self.pose_b,
                Vector::ZERO,
                &self.b,
                options,
            )
            .unwrap()
        }
    }

    impl<A: Shape, B: Shape> Queries for Of<A, B> {
        fn collides(&self) -> bool {
            query::intersection_test(&self.pose_a, &self.a, &self.pose_b, &self.b)
                .unwrap()
                .intersecting
        }

        fn penetrates(&self) -> Option<Vec3> {
            // It can give shapes a hair apart as touching.
            query::contact(&self.pose_a, &self.a, &self.pose_b, &self.b, 0.0)
                .unwrap()
                .filter(|contact| contact.dist <= 0.0)
                .map(|contact| from_parry(contact.normal1 * contact.dist))
        }

        fn toi(&self) -> Option<f32> {
            let options = ShapeCastOptions {
                compute_impact_geometry_on_penetration: false,
                ..ShapeCastOptions::default()
            };
            self.shape_cast(options).map(|hit| hit.time_of_impact)
        }
    }

    pub struct PointAnd<B> {
        p: Vector,
        b: B,
        pose_b: Pose,
        vel: Vector,
    }

    pub fn point_and<B: Make>(pairs: &[Pair]) -> Vec<PointAnd<B>> {
        pairs
            .iter()
            .map(|pair| PointAnd {
                p: to_parry(pair.a.pos),
                b: B::make(&pair.b),
                pose_b: pose(&pair.b),
                vel: to_parry(pair.vel),
            })
            .collect()
    }

    impl<B: Shape> Queries for PointAnd<B> {
        fn collides(&self) -> bool {
            self.b.contains_point(&self.pose_b, self.p)
        }

        fn penetrates(&self) -> Option<Vec3> {
            let projection = self.b.project_point(&self.pose_b, self.p, false);
            projection
                .is_inside
                .then(|| from_parry(projection.point - self.p))
        }

        fn toi(&self) -> Option<f32> {
            let ray = Ray::new(self.p, self.vel);
            self.b.cast_ray(&self.pose_b, &ray, f32::MAX, true)
        }
    }
}

/// The three queries, which each implementation answers in auburn's terms.
trait Queries {
    fn collides(&self) -> bool;
    fn penetrates(&self) -> Option<Vec3>;
    fn toi(&self) -> Option<f32>;
}

/// A formula for `toi`, to name one that is missing.
type Moving<R> = fn(&Body, &Body, Vec3) -> R;
/// A formula for `collides` or `penetrates`, to name one that is missing.
type Overlap<R> = fn(&Body, &Body) -> R;

/// The three queries for one kind of pair: auburn's answers, parry's, and the formulas'.
#[allow(clippy::too_many_arguments)]
fn queries(
    c: &mut Criterion,
    name: &str,
    pairs: &[Pair],
    auburn: &[impl Queries],
    parry: &[impl Queries],
    collides: Option<impl Fn(&Body, &Body) -> bool>,
    penetrates: Option<impl Fn(&Body, &Body) -> Option<Vec3>>,
    toi: Option<impl Fn(&Body, &Body, Vec3) -> Option<f32>>,
) {
    let id = |query: &str| format!("3d/{query}/{name}");
    compare(
        c,
        &id("collides"),
        |i| auburn[i].collides(),
        |i| parry[i].collides(),
        collides.map(|f| move |i: usize| f(&pairs[i].a, &pairs[i].b)),
    );
    compare(
        c,
        &id("penetrates"),
        |i| auburn[i].penetrates(),
        |i| parry[i].penetrates(),
        penetrates.map(|f| move |i: usize| f(&pairs[i].a, &pairs[i].b)),
    );
    compare(
        c,
        &id("toi"),
        |i| auburn[i].toi(),
        |i| parry[i].toi(),
        toi.map(|f| move |i: usize| f(&pairs[i].a, &pairs[i].b, pairs[i].vel)),
    );
}

/// Shapes at `Vec3` positions, which don't turn: the hot path.
fn at_positions(c: &mut Criterion) {
    use formula3d::*;
    use parry3d::shape::{Ball as ParryBall, Cuboid};
    let pairs = &pairs(false);

    queries(
        c,
        "point-ball/Vec3",
        pairs,
        &auburn::of::<Point, Ball, Vec3>(pairs),
        &parry::point_and::<ParryBall>(pairs),
        Some(point_ball_collides),
        Some(point_ball_penetrates),
        Some(point_ball_toi),
    );
    queries(
        c,
        "point-box/Vec3",
        pairs,
        &auburn::of::<Point, Box3d, Vec3>(pairs),
        &parry::point_and::<Cuboid>(pairs),
        Some(point_box_collides),
        Some(point_box_penetrates),
        Some(point_box_toi),
    );
    queries(
        c,
        "ball-ball/Vec3",
        pairs,
        &auburn::of::<Ball, Ball, Vec3>(pairs),
        &parry::of::<ParryBall, ParryBall>(pairs),
        Some(ball_ball_collides),
        Some(ball_ball_penetrates),
        Some(ball_ball_toi),
    );
    queries(
        c,
        "ball-box/Vec3",
        pairs,
        &auburn::of::<Ball, Box3d, Vec3>(pairs),
        &parry::of::<ParryBall, Cuboid>(pairs),
        Some(ball_box_collides),
        Some(ball_box_penetrates),
        None::<Moving<Option<f32>>>,
    );
    queries(
        c,
        "box-box/Vec3",
        pairs,
        &auburn::of::<Box3d, Box3d, Vec3>(pairs),
        &parry::of::<Cuboid, Cuboid>(pairs),
        Some(box_box_collides),
        Some(box_box_penetrates),
        Some(box_box_toi),
    );
}

/// Shapes turned by a `Transform3d`. A ball turns as it is, so its formulas don't change; a
/// point or a ball against a turned box works in the box's frame.
fn turned(c: &mut Criterion) {
    use formula3d::*;
    use parry3d::shape::{Ball as ParryBall, Cuboid};
    let pairs = &pairs(true);

    queries(
        c,
        "point-ball/Transform3d",
        pairs,
        &auburn::of::<Point, Ball, Transform3d>(pairs),
        &parry::point_and::<ParryBall>(pairs),
        Some(point_ball_collides),
        Some(point_ball_penetrates),
        Some(point_ball_toi),
    );
    queries(
        c,
        "point-box/Transform3d",
        pairs,
        &auburn::of::<Point, Box3d, Transform3d>(pairs),
        &parry::point_and::<Cuboid>(pairs),
        Some(point_turned_box_collides),
        Some(point_turned_box_penetrates),
        Some(point_turned_box_toi),
    );
    queries(
        c,
        "ball-ball/Transform3d",
        pairs,
        &auburn::of::<Ball, Ball, Transform3d>(pairs),
        &parry::of::<ParryBall, ParryBall>(pairs),
        Some(ball_ball_collides),
        Some(ball_ball_penetrates),
        Some(ball_ball_toi),
    );
    queries(
        c,
        "ball-box/Transform3d",
        pairs,
        &auburn::of::<Ball, Box3d, Transform3d>(pairs),
        &parry::of::<ParryBall, Cuboid>(pairs),
        Some(ball_turned_box_collides),
        Some(ball_turned_box_penetrates),
        None::<Moving<Option<f32>>>,
    );
    // Two boxes turned different ways have no short formula.
    queries(
        c,
        "box-box/Transform3d",
        pairs,
        &auburn::of::<Box3d, Box3d, Transform3d>(pairs),
        &parry::of::<Cuboid, Cuboid>(pairs),
        None::<Overlap<bool>>,
        None::<Overlap<Option<Vec3>>>,
        None::<Moving<Option<f32>>>,
    );
}

criterion_group! {
    name = benches;
    config = common::criterion();
    targets = at_positions, turned
}
criterion_main!(benches);

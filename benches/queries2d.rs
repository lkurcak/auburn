//! auburn2d's basic queries, next to parry2d's and the formulas a game would write by hand.
//!
//! Every benchmark answers one query for each of [`PAIRS`] pairs of shapes in turn, the same
//! pairs on every run. A point is in the other shape in about one pair in eight, two balls or
//! boxes overlap in about two in five, and from a quarter to two thirds of the casts hit. Its
//! id says which: `2d/<query>/<moving shape>-<other shape>/<transformation>/<implementation>`,
//! the transformation being a `Vec2` position or a turned `Transform2d`.

mod common;
mod formula2d;

use std::f32::consts::TAU;

use auburn2d::Vec2;
use auburn2d::prelude::*;
use criterion::{Criterion, criterion_group, criterion_main};

use common::{PAIRS, Rng, compare};
use formula2d::Body;

/// A pair: `a` moves at the velocity, `b` stands still.
struct Pair {
    a: Body,
    b: Body,
    vel: Vec2,
}

/// The same pairs on every run, turned or not.
fn pairs(turned: bool) -> Vec<Pair> {
    let mut rng = Rng::new(2);
    (0..PAIRS)
        .map(|_| {
            let b = body(&mut rng, Vec2::ZERO, 10.0, turned);
            let a = body(&mut rng, b.pos, 3.0, turned);
            let vel = Vec2::new(rng.range(-8.0, 8.0), rng.range(-8.0, 8.0));
            Pair { a, b, vel }
        })
        .collect()
}

/// A shape up to `spread` from `around` along each axis, of every size at once: its shape
/// takes the radius or the half size it needs.
fn body(rng: &mut Rng, around: Vec2, spread: f32, turned: bool) -> Body {
    let pos = around + Vec2::new(rng.range(-spread, spread), rng.range(-spread, spread));
    let angle = if turned { rng.range(0.0, TAU) } else { 0.0 };
    Body {
        pos,
        angle,
        rot: Vec2::from_angle(angle),
        radius: rng.range(0.5, 1.5),
        half_size: Vec2::new(rng.range(0.5, 1.5), rng.range(0.5, 1.5)),
    }
}

/// The pairs as auburn's shapes and transformations.
mod auburn {
    use super::{Body, Pair, Queries};
    use auburn2d::Vec2;
    use auburn2d::prelude::*;

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

    impl Make for Box2d {
        fn make(body: &Body) -> Self {
            Box2d::new(body.half_size)
        }
    }

    pub trait Place {
        fn place(body: &Body) -> Self;
    }

    impl Place for Vec2 {
        fn place(body: &Body) -> Self {
            body.pos
        }
    }

    impl Place for Transform2d {
        fn place(body: &Body) -> Self {
            Transform2d {
                pos: body.pos,
                rot: Rotor2d::radians(body.angle),
                scale: 1.0,
            }
        }
    }

    pub struct Of<A, B, T> {
        a: A,
        at_a: T,
        b: B,
        at_b: T,
        vel: Vec2,
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
        T: Transformation2d,
    {
        fn collides(&self) -> bool {
            self.a.at(&self.at_a).collides(&self.b.at(&self.at_b))
        }

        fn penetrates(&self) -> Option<Vec2> {
            self.a.at(&self.at_a).penetrates(&self.b.at(&self.at_b))
        }

        fn toi(&self) -> Option<f32> {
            self.a.at(&self.at_a).toi(&self.b.at(&self.at_b), self.vel)
        }

        fn cast(&self) -> Option<CastHit> {
            self.a.at(&self.at_a).cast(&self.b.at(&self.at_b), self.vel)
        }
    }
}

/// The pairs as parry's shapes and poses, with its answers in auburn's terms. A point is a
/// point query or a ray cast, as parry has no point shape.
mod parry {
    use super::{Body, Pair, Queries};
    use auburn2d::Vec2;
    use auburn2d::prelude::CastHit;
    use parry2d::math::{Pose, Vector};
    use parry2d::query::{self, Ray, ShapeCastOptions};
    use parry2d::shape::{Ball, Cuboid, Shape};

    fn to_parry(v: Vec2) -> Vector {
        Vector::new(v.x, v.y)
    }

    fn from_parry(v: Vector) -> Vec2 {
        Vec2::new(v.x, v.y)
    }

    fn pose(body: &Body) -> Pose {
        Pose::new(to_parry(body.pos), body.angle)
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

        fn penetrates(&self) -> Option<Vec2> {
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

        fn cast(&self) -> Option<CastHit> {
            let hit = self.shape_cast(ShapeCastOptions::default())?;
            Some(CastHit {
                time: hit.time_of_impact,
                normal: from_parry(self.pose_b.rotation.transform_vector(hit.normal2)),
                point: from_parry(self.pose_b.transform_point(hit.witness2)),
            })
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

        fn penetrates(&self) -> Option<Vec2> {
            let projection = self.b.project_point(&self.pose_b, self.p, false);
            projection
                .is_inside
                .then(|| from_parry(projection.point - self.p))
        }

        fn toi(&self) -> Option<f32> {
            let ray = Ray::new(self.p, self.vel);
            self.b.cast_ray(&self.pose_b, &ray, f32::MAX, true)
        }

        fn cast(&self) -> Option<CastHit> {
            let ray = Ray::new(self.p, self.vel);
            let hit = self
                .b
                .cast_ray_and_get_normal(&self.pose_b, &ray, f32::MAX, true)?;
            Some(CastHit {
                time: hit.time_of_impact,
                normal: from_parry(hit.normal),
                point: from_parry(ray.point_at(hit.time_of_impact)),
            })
        }
    }
}

/// The four queries, which each implementation answers in auburn's terms.
trait Queries {
    fn collides(&self) -> bool;
    fn penetrates(&self) -> Option<Vec2>;
    fn toi(&self) -> Option<f32>;
    fn cast(&self) -> Option<CastHit>;
}

/// A formula for `collides` or `penetrates`, to name one that is missing.
type Overlap<R> = fn(&Body, &Body) -> R;
/// A formula for `toi` or `cast`, to name one that is missing.
type Moving<R> = fn(&Body, &Body, Vec2) -> R;

/// The four queries for one kind of pair: auburn's answers, parry's, and the formulas'.
#[allow(clippy::too_many_arguments)]
fn queries(
    c: &mut Criterion,
    name: &str,
    pairs: &[Pair],
    auburn: &[impl Queries],
    parry: &[impl Queries],
    collides: Option<impl Fn(&Body, &Body) -> bool>,
    penetrates: Option<impl Fn(&Body, &Body) -> Option<Vec2>>,
    toi: Option<impl Fn(&Body, &Body, Vec2) -> Option<f32>>,
    cast: Option<impl Fn(&Body, &Body, Vec2) -> Option<CastHit>>,
) {
    let id = |query: &str| format!("2d/{query}/{name}");
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
    compare(
        c,
        &id("cast"),
        |i| auburn[i].cast(),
        |i| parry[i].cast(),
        cast.map(|f| move |i: usize| f(&pairs[i].a, &pairs[i].b, pairs[i].vel)),
    );
}

/// Shapes at `Vec2` positions, which don't turn: the hot path.
fn at_positions(c: &mut Criterion) {
    use formula2d::*;
    use parry2d::shape::{Ball as ParryBall, Cuboid};
    let pairs = &pairs(false);

    queries(
        c,
        "point-ball/Vec2",
        pairs,
        &auburn::of::<Point, Ball, Vec2>(pairs),
        &parry::point_and::<ParryBall>(pairs),
        Some(point_ball_collides),
        Some(point_ball_penetrates),
        Some(point_ball_toi),
        Some(point_ball_cast),
    );
    queries(
        c,
        "point-box/Vec2",
        pairs,
        &auburn::of::<Point, Box2d, Vec2>(pairs),
        &parry::point_and::<Cuboid>(pairs),
        Some(point_box_collides),
        Some(point_box_penetrates),
        Some(point_box_toi),
        Some(point_box_cast),
    );
    queries(
        c,
        "ball-ball/Vec2",
        pairs,
        &auburn::of::<Ball, Ball, Vec2>(pairs),
        &parry::of::<ParryBall, ParryBall>(pairs),
        Some(ball_ball_collides),
        Some(ball_ball_penetrates),
        Some(ball_ball_toi),
        Some(ball_ball_cast),
    );
    queries(
        c,
        "ball-box/Vec2",
        pairs,
        &auburn::of::<Ball, Box2d, Vec2>(pairs),
        &parry::of::<ParryBall, Cuboid>(pairs),
        Some(ball_box_collides),
        Some(ball_box_penetrates),
        Some(ball_box_toi),
        Some(ball_box_cast),
    );
    queries(
        c,
        "box-box/Vec2",
        pairs,
        &auburn::of::<Box2d, Box2d, Vec2>(pairs),
        &parry::of::<Cuboid, Cuboid>(pairs),
        Some(box_box_collides),
        Some(box_box_penetrates),
        Some(box_box_toi),
        Some(box_box_cast),
    );
}

/// Shapes turned by a `Transform2d`. A ball turns as it is, so its formulas don't change; a
/// point or a ball against a turned box works in the box's frame.
fn turned(c: &mut Criterion) {
    use formula2d::*;
    use parry2d::shape::{Ball as ParryBall, Cuboid};
    let pairs = &pairs(true);

    queries(
        c,
        "point-ball/Transform2d",
        pairs,
        &auburn::of::<Point, Ball, Transform2d>(pairs),
        &parry::point_and::<ParryBall>(pairs),
        Some(point_ball_collides),
        Some(point_ball_penetrates),
        Some(point_ball_toi),
        Some(point_ball_cast),
    );
    queries(
        c,
        "point-box/Transform2d",
        pairs,
        &auburn::of::<Point, Box2d, Transform2d>(pairs),
        &parry::point_and::<Cuboid>(pairs),
        Some(point_turned_box_collides),
        Some(point_turned_box_penetrates),
        Some(point_turned_box_toi),
        Some(point_turned_box_cast),
    );
    queries(
        c,
        "ball-ball/Transform2d",
        pairs,
        &auburn::of::<Ball, Ball, Transform2d>(pairs),
        &parry::of::<ParryBall, ParryBall>(pairs),
        Some(ball_ball_collides),
        Some(ball_ball_penetrates),
        Some(ball_ball_toi),
        Some(ball_ball_cast),
    );
    queries(
        c,
        "ball-box/Transform2d",
        pairs,
        &auburn::of::<Ball, Box2d, Transform2d>(pairs),
        &parry::of::<ParryBall, Cuboid>(pairs),
        Some(ball_turned_box_collides),
        Some(ball_turned_box_penetrates),
        Some(ball_turned_box_toi),
        Some(ball_turned_box_cast),
    );
    // Two boxes turned different ways have no short formula.
    queries(
        c,
        "box-box/Transform2d",
        pairs,
        &auburn::of::<Box2d, Box2d, Transform2d>(pairs),
        &parry::of::<Cuboid, Cuboid>(pairs),
        None::<Overlap<bool>>,
        None::<Overlap<Option<Vec2>>>,
        None::<Moving<Option<f32>>>,
        None::<Moving<Option<CastHit>>>,
    );
}

criterion_group! {
    name = benches;
    config = common::criterion();
    targets = at_positions, turned
}
criterion_main!(benches);

//! The playground's scene: a point, a ball, a box and a polygon, the query, and the moving
//! shape's velocity.
//!
//! It asks auburn2d the query, and writes itself out two ways: as text for the page's URL,
//! which [`Scene::from_text`] reads back exactly, and as a Rust test.

use auburn2d::prelude::*;
use std::fmt::{self, Write as _};
use std::panic::{AssertUnwindSafe, catch_unwind};

/// The most sides a polygon can have.
pub const MAX_SIDES: u32 = 64;

/// The queries, in the order the side panel lists them.
#[derive(Clone, Copy, Debug, PartialEq, Eq)]
pub enum Query {
    Collides,
    Penetrates,
    Toi,
    Toiae,
    Tttoi,
    Tttoiae,
    Cast,
}

impl Query {
    pub const ALL: [Self; 7] = [
        Self::Collides,
        Self::Penetrates,
        Self::Toi,
        Self::Toiae,
        Self::Tttoi,
        Self::Tttoiae,
        Self::Cast,
    ];

    /// The method's name, as in `a.toi(&b, vel)`.
    pub fn name(self) -> &'static str {
        match self {
            Self::Collides => "collides",
            Self::Penetrates => "penetrates",
            Self::Toi => "toi",
            Self::Toiae => "toiae",
            Self::Tttoi => "tttoi",
            Self::Tttoiae => "tttoiae",
            Self::Cast => "cast",
        }
    }

    /// What it answers, in a sentence.
    pub fn about(self) -> &'static str {
        match self {
            Self::Collides => "Whether the shapes overlap. Touching counts.",
            Self::Penetrates => "How far, and which way, to push the moving shape out.",
            Self::Toi => "When the moving shape first touches the other, from now on.",
            Self::Toiae => "When it starts and stops overlapping the other, from now on.",
            Self::Tttoi => {
                "When it starts to touch the other, as if it had always moved: \
                 negative if that was in the past."
            }
            Self::Tttoiae => "When it starts and stops overlapping the other, over all of time.",
            Self::Cast => {
                "When the moving shape first touches the other, from now on, the normal out of \
                 the other there, and a point where they touch. If they overlap already, the \
                 shortest way out."
            }
        }
    }

    /// Whether it takes a velocity.
    pub fn is_cast(self) -> bool {
        !matches!(self, Self::Collides | Self::Penetrates)
    }

    fn from_name(name: &str) -> Option<Self> {
        Self::ALL.into_iter().find(|q| q.name() == name)
    }
}

/// A shape, with its size in its own frame.
#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Form {
    Point,
    Ball {
        radius: f32,
    },
    Box {
        half_size: Vec2,
    },
    /// `Poly2d::regular(sides, radius)`.
    Polygon {
        sides: u32,
        radius: f32,
    },
}

impl Form {
    pub fn name(self) -> &'static str {
        match self {
            Self::Point => "point",
            Self::Ball { .. } => "ball",
            Self::Box { .. } => "box",
            Self::Polygon { .. } => "polygon",
        }
    }

    /// Its variable in a test: `box` is a keyword.
    fn variable(self) -> &'static str {
        match self {
            Self::Box { .. } => "box2d",
            form => form.name(),
        }
    }

    /// How far out the turn-and-scale handle sits, in the shape's frame.
    pub fn reach(self) -> f32 {
        let size = match self {
            Self::Point => 1.0,
            Self::Ball { radius } | Self::Polygon { radius, .. } => radius,
            Self::Box { half_size } => half_size.x,
        };
        size.abs().max(0.5)
    }

    /// The vertices of a box or a polygon in its own frame, counter-clockwise, as auburn2d
    /// has them: none for a point, a ball, or a polygon with no sides.
    pub fn vertices(self) -> Vec<Vec2> {
        match self {
            Self::Point | Self::Ball { .. } => Vec::new(),
            Self::Box { half_size } => Poly2d::from(Box2d::new(half_size)).points.to_vec(),
            Self::Polygon { sides, radius } => Poly2d::regular(sides, radius).points.to_vec(),
        }
    }

    fn code(self) -> String {
        match self {
            Self::Point => "Point".into(),
            Self::Ball { radius } => format!("Ball::new({})", lit(radius)),
            Self::Box { half_size } => format!("Box2d::new({})", vec2(half_size)),
            Self::Polygon { sides, radius } => format!("Poly2d::regular({sides}, {})", lit(radius)),
        }
    }
}

/// Builds `$form`'s auburn2d shape as `$shape`, and gives `$body` with it.
macro_rules! with_shape {
    ($form:expr, |$shape:ident| $body:expr) => {
        match $form {
            Form::Point => {
                let $shape = Point;
                $body
            }
            Form::Ball { radius } => {
                let $shape = Ball::new(radius);
                $body
            }
            Form::Box { half_size } => {
                let $shape = Box2d::new(half_size);
                $body
            }
            Form::Polygon { sides, radius } => {
                let $shape = Poly2d::regular(sides, radius);
                $body
            }
        }
    };
}

/// A shape and where it is: translated, turned by `angle` (in radians) and scaled.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct Body {
    pub form: Form,
    pub pos: Vec2,
    pub angle: f32,
    pub scale: f32,
}

impl Body {
    const fn new(form: Form, x: f32, y: f32) -> Self {
        Self {
            form,
            pos: Vec2::new(x, y),
            angle: 0.0,
            scale: 1.0,
        }
    }

    /// Its transformation, built the way the test that [`Scene::test_code`] writes builds it.
    pub fn transform(&self) -> Transform2d {
        let mut t = Transform2d::from_translation(self.pos);
        if self.angle != 0.0 {
            t = t.with_angle(self.angle);
        }
        if self.scale != 1.0 {
            t = t.with_scale(self.scale);
        }
        t
    }

    fn transform_code(&self) -> String {
        let mut code = format!("Transform2d::from_translation({})", vec2(self.pos));
        if self.angle != 0.0 {
            let _ = write!(code, ".with_angle({})", lit(self.angle));
        }
        if self.scale != 1.0 {
            let _ = write!(code, ".with_scale({})", lit(self.scale));
        }
        code
    }
}

/// What a query gave.
#[derive(Clone, Debug, PartialEq)]
pub enum Answer {
    Collides(bool),
    Penetrates(Option<Vec2>),
    Time(Option<f32>),
    Span(Option<(f32, f32)>),
    Hit(Option<CastHit>),
    /// The query panicked, with this message.
    Panicked(String),
}

impl Answer {
    /// Whether the shapes meet: they collide, or the query has an answer.
    pub fn hit(&self) -> bool {
        match self {
            Self::Collides(hit) => *hit,
            Self::Penetrates(push) => push.is_some(),
            Self::Time(t) => t.is_some(),
            Self::Span(span) => span.is_some(),
            Self::Hit(hit) => hit.is_some(),
            Self::Panicked(_) => false,
        }
    }

    /// An assertion that `r` is this answer.
    fn assertion(&self) -> String {
        if self.to_string().contains("NaN") {
            return format!("assert_eq!(format!(\"{{r:?}}\"), {:?});", self.to_string());
        }
        let some = |value: String| format!("assert_eq!(r, Some({value}));");
        match *self {
            Self::Collides(true) => "assert!(r);".into(),
            Self::Collides(false) => "assert!(!r);".into(),
            Self::Penetrates(Some(push)) => some(vec2(push)),
            Self::Time(Some(t)) => some(lit(t)),
            Self::Span(Some((entry, exit))) => some(format!("({}, {})", lit(entry), lit(exit))),
            Self::Hit(Some(hit)) => some(format!(
                "CastHit {{ time: {}, normal: {}, point: {} }}",
                lit(hit.time),
                vec2(hit.normal),
                vec2(hit.point)
            )),
            Self::Penetrates(None) | Self::Time(None) | Self::Span(None) | Self::Hit(None) => {
                "assert!(r.is_none(), \"{r:?}\");".into()
            }
            Self::Panicked(_) => unreachable!("a panic has no assertion"),
        }
    }
}

/// As Rust's `Debug` has it.
impl fmt::Display for Answer {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Collides(hit) => write!(f, "{hit}"),
            Self::Penetrates(push) => write!(f, "{push:?}"),
            Self::Time(t) => write!(f, "{t:?}"),
            Self::Span(span) => write!(f, "{span:?}"),
            Self::Hit(hit) => write!(f, "{hit:?}"),
            Self::Panicked(message) => write!(f, "panicked: {message}"),
        }
    }
}

#[derive(Clone, Debug, PartialEq)]
pub struct Scene {
    /// A point, a ball, a box and a polygon, in that order.
    pub bodies: [Body; 4],
    pub query: Query,
    /// Which body moves: the query asks about it against each of the others.
    pub moving: usize,
    /// The moving body's velocity, for the casts.
    pub vel: Vec2,
}

impl Default for Scene {
    fn default() -> Self {
        let mut tilted_box = Body::new(
            Form::Box {
                half_size: Vec2::new(1.5, 0.75),
            },
            2.0,
            -1.0,
        );
        tilted_box.angle = 0.4;
        Self {
            bodies: [
                Body::new(Form::Point, -3.0, 2.0),
                Body::new(Form::Ball { radius: 1.0 }, -4.0, -1.0),
                tilted_box,
                Body::new(
                    Form::Polygon {
                        sides: 5,
                        radius: 1.25,
                    },
                    2.0,
                    2.5,
                ),
            ],
            query: Query::Toiae,
            moving: 1,
            vel: Vec2::new(4.0, 0.0),
        }
    }
}

impl Scene {
    /// The bodies other than the moving one.
    pub fn others(&self) -> impl DoubleEndedIterator<Item = usize> + use<> {
        let moving = self.moving;
        (0..4).filter(move |&i| i != moving)
    }

    /// The query's answer for the moving body against `target`.
    pub fn answer(&self, target: usize) -> Answer {
        let (a, b) = (&self.bodies[self.moving], &self.bodies[target]);
        let (at_a, at_b) = (a.transform(), b.transform());
        let (query, vel) = (self.query, self.vel);
        catch_unwind(AssertUnwindSafe(|| {
            with_shape!(a.form, |a| with_shape!(b.form, |b| ask(
                query,
                &a.at(&at_a),
                &b.at(&at_b),
                vel
            )))
        }))
        .unwrap_or_else(|panic| {
            let message = (panic.downcast_ref::<&str>().map(|s| s.to_string()))
                .or_else(|| panic.downcast_ref::<String>().cloned())
                .unwrap_or_default();
            Answer::Panicked(message)
        })
    }

    /// A Rust test that asks auburn2d the query for the moving body against `target`, and
    /// asserts the answer it gives now: if that's wrong, the right answer goes there instead.
    /// It needs `use auburn2d::prelude::*`, as `auburn2d/tests/regressions.rs` has.
    pub fn test_code(&self, target: usize) -> String {
        let (a, b) = (self.bodies[self.moving], self.bodies[target]);
        let (query, answer) = (self.query.name(), self.answer(target));
        let mut code = String::new();
        let mut line = |text: String| {
            code.push_str(&text);
            code.push('\n');
        };
        line("#[test]".into());
        line(format!(
            "fn {}_{query}_{}() {{",
            a.form.name(),
            b.form.name()
        ));
        line(format!("    // Playground: {}", self.to_text()));
        for body in [a, b] {
            let name = body.form.variable();
            line(format!("    let {name} = {};", body.form.code()));
            line(format!("    let {name}_at = {};", body.transform_code()));
        }
        let vel = if self.query.is_cast() {
            line(format!("    let vel = {};", vec2(self.vel)));
            ", vel"
        } else {
            ""
        };
        let (a, b) = (a.form.variable(), b.form.variable());
        let call = format!("{a}.at(&{a}_at).{query}(&{b}.at(&{b}_at){vel})");
        let version = env!("CARGO_PKG_VERSION");
        if let Answer::Panicked(message) = &answer {
            line(format!("    // auburn2d {version} panics here: {message}"));
            line(format!("    let _ = {call};"));
        } else {
            line(format!("    let r = {call};"));
            line(format!(
                "    // auburn2d {version} gives {answer}. If that's wrong, assert the right answer."
            ));
            line(format!("    {}", answer.assertion()));
        }
        line("}".into());
        code
    }

    /// The scene as text, for the page's URL: `q=toiae&m=ball&v=4,0&point=x,y,angle,scale`, then
    /// `ball=` with the radius after the scale, `box=` with the half size, and `polygon=` with
    /// the sides and the radius. Every number reads back exactly.
    pub fn to_text(&self) -> String {
        let moving = self.bodies[self.moving].form.name();
        let mut text = format!(
            "q={}&m={moving}&v={},{}",
            self.query.name(),
            self.vel.x,
            self.vel.y
        );
        for body in &self.bodies {
            let (name, pos) = (body.form.name(), body.pos);
            let _ = write!(
                text,
                "&{name}={},{},{},{}",
                pos.x, pos.y, body.angle, body.scale
            );
            let _ = match body.form {
                Form::Point => Ok(()),
                Form::Ball { radius } => write!(text, ",{radius}"),
                Form::Box { half_size } => write!(text, ",{},{}", half_size.x, half_size.y),
                Form::Polygon { sides, radius } => write!(text, ",{sides},{radius}"),
            };
        }
        text
    }

    /// Reads [`Self::to_text`]'s text back: `None` if a part of it doesn't read. Parts it leaves
    /// out keep their defaults, and parts it doesn't know are skipped, so older links open.
    pub fn from_text(text: &str) -> Option<Self> {
        let mut scene = Self::default();
        for part in text.split('&').filter(|part| !part.is_empty()) {
            let (key, value) = part.split_once('=')?;
            match key {
                "q" => scene.query = Query::from_name(value)?,
                "m" => scene.moving = scene.index(value)?,
                "v" => {
                    let [x, y] = numbers(value)?;
                    scene.vel = Vec2::new(x, y);
                }
                _ => {
                    if let Some(i) = scene.index(key) {
                        scene.bodies[i] = read_body(scene.bodies[i].form, value)?;
                    }
                }
            }
        }
        Some(scene)
    }

    /// Reads a scene from a playground link, or from the part after its `#`.
    pub fn from_link(link: &str) -> Option<Self> {
        let link = link.trim();
        Self::from_text(link.rsplit_once('#').map_or(link, |(_, text)| text))
    }

    /// The index of the body with this form's name.
    fn index(&self, name: &str) -> Option<usize> {
        self.bodies.iter().position(|body| body.form.name() == name)
    }
}

fn ask<S1, S2>(
    query: Query,
    a: &Collider<'_, S1, Transform2d>,
    b: &Collider<'_, S2, Transform2d>,
    vel: Vec2,
) -> Answer
where
    S1: Shape + Support,
    S2: Shape + Support,
{
    match query {
        Query::Collides => Answer::Collides(a.collides(b)),
        Query::Penetrates => Answer::Penetrates(a.penetrates(b)),
        Query::Toi => Answer::Time(a.toi(b, vel)),
        Query::Toiae => Answer::Span(a.toiae(b, vel)),
        Query::Tttoi => Answer::Time(a.tttoi(b, vel)),
        Query::Tttoiae => Answer::Span(a.tttoiae(b, vel)),
        Query::Cast => Answer::Hit(a.cast(b, vel)),
    }
}

/// Reads `x,y,angle,scale` and the numbers `form` adds after them.
fn read_body(form: Form, text: &str) -> Option<Body> {
    let parts: Vec<&str> = text.split(',').collect();
    let extra = match form {
        Form::Point => 0,
        Form::Ball { .. } => 1,
        Form::Box { .. } | Form::Polygon { .. } => 2,
    };
    if parts.len() != 4 + extra {
        return None;
    }
    let number = |i: usize| parts[i].parse::<f32>().ok();
    let form = match form {
        Form::Point => Form::Point,
        Form::Ball { .. } => Form::Ball { radius: number(4)? },
        Form::Box { .. } => Form::Box {
            half_size: Vec2::new(number(4)?, number(5)?),
        },
        Form::Polygon { .. } => Form::Polygon {
            sides: parts[4].parse().ok().filter(|&sides| sides <= MAX_SIDES)?,
            radius: number(5)?,
        },
    };
    Some(Body {
        form,
        pos: Vec2::new(number(0)?, number(1)?),
        angle: number(2)?,
        scale: number(3)?,
    })
}

/// Exactly `N` numbers, separated by commas.
fn numbers<const N: usize>(text: &str) -> Option<[f32; N]> {
    let mut parts = text.split(',');
    let mut out = [0.0; N];
    for x in &mut out {
        *x = parts.next()?.parse().ok()?;
    }
    parts.next().is_none().then_some(out)
}

/// `x` as a Rust expression that gives it back exactly.
fn lit(x: f32) -> String {
    if x.is_nan() {
        "f32::NAN".into()
    } else if x == f32::INFINITY {
        "f32::INFINITY".into()
    } else if x == f32::NEG_INFINITY {
        "f32::NEG_INFINITY".into()
    } else {
        format!("{x:?}")
    }
}

fn vec2(v: Vec2) -> String {
    format!("Vec2::new({}, {})", lit(v.x), lit(v.y))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn text_reads_back_exactly() {
        let mut scene = Scene {
            query: Query::Tttoi,
            moving: 3,
            vel: Vec2::new(1e-7, -0.1),
            ..Scene::default()
        };
        scene.bodies[0].pos = Vec2::new(1.0 / 3.0, -2.5e10);
        scene.bodies[1].angle = 0.1;
        scene.bodies[2].scale = -1.5;
        scene.bodies[3].form = Form::Polygon {
            sides: 7,
            radius: 0.3,
        };
        assert_eq!(Scene::from_text(&scene.to_text()), Some(scene.clone()));
        let link = format!("https://example.com/auburn/#{}", scene.to_text());
        assert_eq!(Scene::from_link(&link), Some(scene));
    }

    #[test]
    fn text_keeps_defaults_and_skips_unknown_parts() {
        assert_eq!(Scene::from_text(""), Some(Scene::default()));
        assert_eq!(Scene::from_text("later=1"), Some(Scene::default()));
        let scene = Scene::from_text("q=collides&m=box").unwrap();
        assert_eq!((scene.query, scene.moving), (Query::Collides, 2));
    }

    #[test]
    fn text_that_does_not_read() {
        for text in [
            "q=nope",
            "m=triangle",
            "v=1",
            "v=1,2,3",
            "ball=1,2,0,1",
            "ball=1,2,0,1,x",
            "polygon=0,0,0,1,65,1",
            "q",
        ] {
            assert_eq!(Scene::from_text(text), None, "{text}");
        }
    }

    #[test]
    fn answers() {
        let scene = Scene::default();
        // The ball moves right at 4 into the box, and misses the point and the polygon.
        let answers: Vec<_> = scene.others().map(|i| scene.answer(i).hit()).collect();
        assert_eq!(answers, [false, true, false]);
    }

    #[test]
    fn test_code() {
        let scene = Scene::default();
        let code = scene.test_code(2);
        let answer = scene.answer(2);
        let Answer::Span(Some((entry, exit))) = answer else {
            panic!("{answer:?}");
        };
        let lines = [
            "fn ball_toiae_box() {".into(),
            "    let ball = Ball::new(1.0);".into(),
            "    let ball_at = Transform2d::from_translation(Vec2::new(-4.0, -1.0));".into(),
            "    let box2d = Box2d::new(Vec2::new(1.5, 0.75));".into(),
            "    let box2d_at = Transform2d::from_translation(Vec2::new(2.0, -1.0)).with_angle(0.4);"
                .into(),
            "    let vel = Vec2::new(4.0, 0.0);".into(),
            "    let r = ball.at(&ball_at).toiae(&box2d.at(&box2d_at), vel);".into(),
            format!("    assert_eq!(r, Some(({entry:?}, {exit:?})));"),
        ];
        for line in lines {
            assert!(code.lines().any(|l| l == line), "no {line:?} in\n{code}");
        }
    }

    #[test]
    fn assertions() {
        let cases = [
            (Answer::Collides(false), "assert!(!r);"),
            (Answer::Time(None), "assert!(r.is_none(), \"{r:?}\");"),
            (
                Answer::Hit(Some(CastHit {
                    time: 0.5,
                    normal: Vec2::NEG_X,
                    point: Vec2::new(3.0, -1.0),
                })),
                "assert_eq!(r, Some(CastHit { time: 0.5, normal: Vec2::new(-1.0, 0.0), \
                 point: Vec2::new(3.0, -1.0) }));",
            ),
            (
                Answer::Span(Some((f32::NEG_INFINITY, 1e-7))),
                "assert_eq!(r, Some((f32::NEG_INFINITY, 1e-7)));",
            ),
            (
                Answer::Penetrates(Some(Vec2::new(f32::NAN, 1.0))),
                "assert_eq!(format!(\"{r:?}\"), \"Some(Vec2(NaN, 1.0))\");",
            ),
        ];
        for (answer, assertion) in cases {
            assert_eq!(answer.assertion(), assertion);
        }
    }
}

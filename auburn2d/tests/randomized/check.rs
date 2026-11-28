//! The checks. Each one runs seeded cases for one pair of shapes under one transformation
//! type and compares the library with the ground truth.

use crate::sample::{Config, Place, Rng, Sample, velocity, wide};
use crate::truth::{Geometry, Truth, contact, depth_of};
use auburn2d::prelude::*;
use glam::DVec2;
use std::fmt::Write;

/// How many cases each check runs: `base`, times `AUBURN_RANDOMIZED_SCALE` when it is set.
pub fn cases(base: usize) -> usize {
    let scale = std::env::var("AUBURN_RANDOMIZED_SCALE").map_or(1, |s| {
        s.parse()
            .expect("AUBURN_RANDOMIZED_SCALE must be a positive integer")
    });
    base * scale
}

/// One check's result.
pub struct Outcome {
    label: String,
    wanted: usize,
    cases: usize,
    skipped: usize,
    failures: usize,
    examples: Vec<String>,
}

impl Outcome {
    fn new(label: String, wanted: usize) -> Self {
        Self {
            label,
            wanted,
            cases: 0,
            skipped: 0,
            failures: 0,
            examples: Vec::new(),
        }
    }

    fn fail(&mut self, example: String) {
        self.failures += 1;
        if self.examples.len() < 3 {
            self.examples.push(example);
        }
    }

    fn is_ok(&self) -> bool {
        // A check that skips most of its cases tests nothing.
        self.failures == 0 && 2 * self.cases >= self.wanted
    }
}

/// Collects outcomes and fails the test with all of them at once.
#[derive(Default)]
pub struct Report(Vec<Outcome>);

impl Report {
    pub fn push(&mut self, outcome: Outcome) {
        self.0.push(outcome);
    }

    pub fn finish(self) {
        let bad: Vec<_> = self.0.iter().filter(|o| !o.is_ok()).collect();
        if bad.is_empty() {
            return;
        }
        let mut msg = format!("{} of {} checks failed\n", bad.len(), self.0.len());
        for o in bad {
            let _ = writeln!(
                msg,
                "\n{}: {} of {} cases wrong ({} skipped as too close to call)",
                o.label, o.failures, o.cases, o.skipped
            );
            for e in &o.examples {
                let _ = writeln!(msg, "  {e}");
            }
        }
        panic!("{msg}");
    }
}

/// The check's name, which also seeds its cases. `a` is placed by a `TA` and `b` by a `TB`.
fn label<A: Sample, B: Sample, TA: Place, TB: Place>(check: &str, cfg: &Config) -> String {
    let place = if TA::NAME == TB::NAME {
        TA::NAME.to_string()
    } else {
        format!("{} and {}", TA::NAME, TB::NAME)
    };
    format!("{check} {}-{} @{place} ({})", A::NAME, B::NAME, cfg.name)
}

struct Scene<A, B, TA, TB> {
    a: A,
    b: B,
    ta: TA,
    tb: TB,
}

impl<A: Sample, B: Sample, TA: Place, TB: Place> Scene<A, B, TA, TB> {
    fn sample(rng: &mut Rng, cfg: &Config) -> Self {
        Self {
            a: A::sample(rng, cfg),
            b: B::sample(rng, cfg),
            ta: TA::sample(rng, cfg),
            tb: TB::sample(rng, cfg),
        }
    }

    /// The truth about moving `a`.
    fn truth(&self) -> Truth {
        Truth::new(&self.a.geometry(&self.ta), &self.b.geometry(&self.tb))
    }

    /// The truth about moving `b`.
    fn reverse_truth(&self) -> Truth {
        Truth::new(&self.b.geometry(&self.tb), &self.a.geometry(&self.ta))
    }

    fn describe(&self) -> String {
        format!(
            "a = {:?} at {:?}, b = {:?} at {:?}",
            self.a, self.ta, self.b, self.tb
        )
    }
}

/// `collides` agrees with the truth in both directions, and `penetrates` returns a finite
/// vector exactly when `collides` is true.
pub fn collides<A: Sample, B: Sample, T: Place>(cfg: &Config, n: usize) -> Outcome
where
    for<'a> Collider<'a, A, T>: Collides<Collider<'a, B, T>> + Penetrates<Collider<'a, B, T>>,
    for<'a> Collider<'a, B, T>: Collides<Collider<'a, A, T>> + Penetrates<Collider<'a, A, T>>,
{
    let label = label::<A, B, T, T>("collides", cfg);
    let mut rng = Rng::from_label(&label);
    let mut out = Outcome::new(label, n);
    let tol = cfg.tolerance();
    for _ in 0..n {
        let s = Scene::<A, B, T, T>::sample(&mut rng, cfg);
        let (ca, cb) = (s.a.at(&s.ta), s.b.at(&s.tb));
        let depth = s.truth().depth(DVec2::ZERO);
        let (ab, ba) = (ca.collides(&cb), cb.collides(&ca));
        let (pab, pba) = (ca.penetrates(&cb), cb.penetrates(&ca));
        let consistent = pab.is_some() == ab
            && pba.is_some() == ba
            && pab.is_none_or(|p| p.is_finite())
            && pba.is_none_or(|p| p.is_finite());
        if depth.abs() <= tol && consistent {
            out.skipped += 1;
            continue;
        }
        out.cases += 1;
        let truth = depth > 0.0;
        if (depth.abs() > tol && (ab != truth || ba != truth)) || !consistent {
            out.fail(format!(
                "{}: depth {depth:.6}, a.collides(b) = {ab}, b.collides(a) = {ba}, \
                 a.penetrates(b) = {pab:?}, b.penetrates(a) = {pba:?}",
                s.describe()
            ));
        }
    }
    out
}

/// For overlapping shapes, `penetrates` returns the shortest vector that separates them, in
/// both directions.
pub fn penetrates<A: Sample, B: Sample, T: Place>(cfg: &Config, n: usize) -> Outcome
where
    for<'a> Collider<'a, A, T>: Penetrates<Collider<'a, B, T>>,
    for<'a> Collider<'a, B, T>: Penetrates<Collider<'a, A, T>>,
{
    let label = label::<A, B, T, T>("penetrates", cfg);
    let mut rng = Rng::from_label(&label);
    let mut out = Outcome::new(label, n);
    let tol = cfg.tolerance();
    // Only overlapping cases count, so draw until there are enough.
    for _ in 0..50 * n {
        if out.cases == n {
            break;
        }
        let s = Scene::<A, B, T, T>::sample(&mut rng, cfg);
        let truth = s.truth();
        let depth = truth.depth(DVec2::ZERO);
        if depth <= tol {
            continue;
        }
        out.cases += 1;
        let (ca, cb) = (s.a.at(&s.ta), s.b.at(&s.tb));
        let slack = 5.0 * tol + 0.01 * depth;
        let directions = [
            ("a.penetrates(b)", ca.penetrates(&cb), truth),
            ("b.penetrates(a)", cb.penetrates(&ca), s.reverse_truth()),
        ];
        for (call, p, truth) in directions {
            let ok = p.is_some_and(|p| {
                let p = wide(p);
                (p.length() - depth).abs() <= slack && truth.depth(p).abs() <= slack
            });
            if !ok {
                let after = p.map(|p| truth.depth(wide(p)));
                out.fail(format!(
                    "{}: depth {depth:.6}, {call} = {p:?} (length {:?}), depth left after \
                     moving by it {after:?}",
                    s.describe(),
                    p.map(Vec2::length),
                ));
            }
        }
    }
    out
}

#[derive(Clone, Copy, Debug, PartialEq)]
pub enum Kind {
    Toi,
    TtToi,
    Toiae,
    TtToiae,
    Cast,
}

impl Kind {
    /// Whether the query only looks forward in time.
    fn bounded(self) -> bool {
        matches!(self, Kind::Toi | Kind::Toiae | Kind::Cast)
    }
}

/// A cast's answer.
#[derive(Clone, Copy, Debug)]
pub enum Hit {
    Time(Option<f32>),
    Span(Option<(f32, f32)>),
    /// `cast`'s hit, and the time `toi` gives, which should be the hit's.
    Contact(Option<CastHit>, Option<f32>),
}

impl Hit {
    /// The hit's time, as `Time` has it, for `Contact`.
    fn time(self) -> Option<Option<f32>> {
        match self {
            Hit::Contact(hit, _) => Some(hit.map(|hit| hit.time)),
            _ => None,
        }
    }
}

/// Why `hit` doesn't touch the way it says, if it doesn't: its normal is a unit vector out of
/// `b`, and once `a` has moved to where they touch, the line across it through `point` keeps
/// them apart, and `point` is on both. Where they overlap at time 0, `a` moves out along the
/// normal by the depth: if the normal isn't the shortest way out, they still overlap then, and
/// no line across it keeps them apart.
fn contact_error(
    hit: CastHit,
    vel: Vec2,
    truth: &Truth,
    (a, b): (&Geometry, &Geometry),
    slack: f64,
) -> Option<String> {
    let n = wide(hit.normal);
    if !hit.point.is_finite() || (n.length() - 1.0).abs() > 1e-5 {
        return Some(format!("normal of length {}", n.length()));
    }
    let shift = if hit.time > 0.0 {
        wide(vel) * f64::from(hit.time)
    } else {
        n * truth.depth(DVec2::ZERO).max(0.0)
    };
    let gap = truth.gap(n, shift);
    let p = wide(hit.point);
    let moved = Geometry {
        points: a.points.iter().map(|&q| q + shift).collect(),
        radius: a.radius,
    };
    let (in_a, in_b) = (depth_of(p, &moved), depth_of(p, b));
    // Where a point or a ball touches, only one point will do.
    let off = contact(&moved, b).map_or(0.0, |q| (p - q).length());
    (gap < -slack || in_a < -slack || in_b < -slack || off > slack).then(|| {
        format!(
            "the line across the normal cuts {:.3e} into them, the point is {:.3e} outside a \
             and {:.3e} outside b, and {off:.3e} from where they touch",
            -gap, -in_a, -in_b
        )
    })
}

fn close(got: f32, want: f64, slack: f64) -> bool {
    (f64::from(got) - want).abs() <= slack + 1e-4 * want.abs()
}

/// The cast `kind`, with `a` moving at a nonzero velocity, agrees with the truth. `a` is placed
/// by a `TA` and `b` by a `TB`.
pub fn cast<A: Sample, B: Sample, TA: Place, TB: Place>(
    cfg: &Config,
    n: usize,
    kind: Kind,
    query: impl Fn(&Collider<A, TA>, &Collider<B, TB>, Vec2) -> Hit,
) -> Outcome {
    let label = label::<A, B, TA, TB>(&format!("{kind:?}"), cfg);
    let mut rng = Rng::from_label(&label);
    let mut out = Outcome::new(label, n);
    let tol = cfg.tolerance();
    for _ in 0..n {
        let s = Scene::<A, B, TA, TB>::sample(&mut rng, cfg);
        let vel = velocity(
            &mut rng,
            cfg,
            s.ta.apply_to_origin(),
            s.tb.apply_to_origin(),
        );
        let got = query(&s.a.at(&s.ta), &s.b.at(&s.tb), vel);
        let sweep = s.truth().sweep(wide(vel));
        let slack = 5.0 * tol / wide(vel).length();
        // A graze, or a bounded cast that starts just as the shapes separate, could go either way.
        let leaving_now =
            kind.bounded() && sweep.overlap.is_some_and(|(_, exit)| exit.abs() < slack);
        if sweep.max_depth.abs() < tol || leaving_now {
            out.skipped += 1;
            continue;
        }
        out.cases += 1;
        let want = match kind {
            Kind::Toi | Kind::Toiae | Kind::Cast => sweep
                .overlap
                .filter(|&(_, exit)| exit >= 0.0)
                .map(|(entry, exit)| (entry.max(0.0), exit)),
            Kind::TtToi | Kind::TtToiae => sweep.overlap,
        };
        let time = |t: Option<f32>| match (t, want) {
            (None, None) => true,
            (Some(t), Some((entry, _))) => close(t, entry, slack),
            _ => false,
        };
        let mut error = None;
        let ok = match got {
            Hit::Time(t) => time(t),
            Hit::Span(span) => match (span, want) {
                (None, None) => true,
                (Some((entry, exit)), Some((want_entry, want_exit))) => {
                    close(entry, want_entry, slack) && close(exit, want_exit, slack)
                }
                _ => false,
            },
            Hit::Contact(hit, toi) => {
                let shapes = (&s.a.geometry(&s.ta), &s.b.geometry(&s.tb));
                error = hit.and_then(|hit| contact_error(hit, vel, &s.truth(), shapes, tol));
                time(toi) && got.time() == Some(toi) && error.is_none()
            }
        };
        if !ok {
            out.fail(format!(
                "{}, vel = {vel:?}: got {got:?}, want (entry, exit) = {want:?}{}",
                s.describe(),
                error.map_or(String::new(), |e| format!(": {e}"))
            ));
        }
    }
    out
}

/// The cast `kind` at zero velocity reports the overlap the shapes have now: `None` if they are
/// apart, and if they overlap, an overlap that lasts forever: from 0 for the bounded casts and
/// from minus infinity for the time-traveling ones, to infinity.
pub fn cast_at_rest<A: Sample, B: Sample, TA: Place, TB: Place>(
    cfg: &Config,
    n: usize,
    kind: Kind,
    query: impl Fn(&Collider<A, TA>, &Collider<B, TB>, Vec2) -> Hit,
) -> Outcome {
    let label = label::<A, B, TA, TB>(&format!("{kind:?} at rest"), cfg);
    let mut rng = Rng::from_label(&label);
    let mut out = Outcome::new(label, n);
    let tol = cfg.tolerance();
    for _ in 0..n {
        let s = Scene::<A, B, TA, TB>::sample(&mut rng, cfg);
        let vel = Vec2::new(rng.zero(), rng.zero());
        let got = query(&s.a.at(&s.ta), &s.b.at(&s.tb), vel);
        let depth = s.truth().depth(DVec2::ZERO);
        if depth.abs() < tol {
            out.skipped += 1;
            continue;
        }
        out.cases += 1;
        let entry = if kind.bounded() {
            0.0
        } else {
            f32::NEG_INFINITY
        };
        let want = (depth > 0.0).then_some((entry, f32::INFINITY));
        let mut error = None;
        let ok = match got {
            Hit::Time(t) => t == want.map(|(entry, _)| entry),
            Hit::Span(span) => span == want,
            Hit::Contact(hit, toi) => {
                let shapes = (&s.a.geometry(&s.ta), &s.b.geometry(&s.tb));
                error = hit.and_then(|hit| contact_error(hit, vel, &s.truth(), shapes, tol));
                toi == want.map(|(entry, _)| entry) && got.time() == Some(toi) && error.is_none()
            }
        };
        if !ok {
            out.fail(format!(
                "{}, vel = {vel:?}: got {got:?}, depth {depth:.6}{}",
                s.describe(),
                error.map_or(String::new(), |e| format!(": {e}"))
            ));
        }
    }
    out
}

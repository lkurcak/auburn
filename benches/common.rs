//! What the 2D and 3D benchmarks share: drawing the pairs, timing a query over them, and
//! checking first that every implementation gives the same answers.

use std::fmt::Debug;
use std::hint::black_box;
use std::time::Duration;

use criterion::{Bencher, Criterion};

/// How many pairs each benchmark draws and goes through in turn: few enough to stay in the
/// cache, as a game's hot loop would, and enough that the hits and misses don't repeat in a
/// pattern the branch predictor could learn.
pub const PAIRS: usize = 1024;

/// Criterion's settings for every benchmark: these are short, steady loops, and there are a
/// couple of hundred of them.
pub fn criterion() -> Criterion {
    Criterion::default()
        .warm_up_time(Duration::from_millis(500))
        .measurement_time(Duration::from_secs(1))
}

/// A small deterministic generator (SplitMix64), so that every run draws the same pairs.
pub struct Rng(u64);

impl Rng {
    pub fn new(seed: u64) -> Self {
        Self(seed)
    }

    fn next(&mut self) -> u64 {
        self.0 = self.0.wrapping_add(0x9e37_79b9_7f4a_7c15);
        let mut z = self.0;
        z = (z ^ (z >> 30)).wrapping_mul(0xbf58_476d_1ce4_e5b9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94d0_49bb_1331_11eb);
        z ^ (z >> 31)
    }

    /// Uniform in `lo..hi`.
    pub fn range(&mut self, lo: f32, hi: f32) -> f32 {
        let unit = (self.next() >> 40) as f32 / (1u64 << 24) as f32;
        lo + (hi - lo) * unit
    }
}

/// Answers close enough to count as the same, as the implementations round differently and
/// some iterate to a tolerance: times to 1e-3, and vectors to 1e-2 of their length, as auburn's
/// `penetrates` goes through EPA, whose directions on curved shapes are good to about 3e-3. A
/// formula with a wrong sign or the wrong side is off by far more.
pub trait Close {
    fn close(&self, other: &Self) -> bool;
}

impl Close for bool {
    fn close(&self, other: &Self) -> bool {
        self == other
    }
}

impl Close for f32 {
    fn close(&self, other: &Self) -> bool {
        (self - other).abs() <= 1e-3 * (1.0 + self.abs().max(other.abs()))
    }
}

impl<T: Close> Close for Option<T> {
    fn close(&self, other: &Self) -> bool {
        match (self, other) {
            (Some(a), Some(b)) => a.close(b),
            (None, None) => true,
            _ => false,
        }
    }
}

impl Close for auburn2d::Vec2 {
    fn close(&self, other: &Self) -> bool {
        self.distance(*other) <= 1e-2 * (1.0 + self.length().max(other.length()))
    }
}

impl Close for auburn3d::Vec3 {
    fn close(&self, other: &Self) -> bool {
        self.distance(*other) <= 1e-2 * (1.0 + self.length().max(other.length()))
    }
}

/// The time, and the normal unless the shapes overlap from the start: parry gives none for a
/// point inside a shape. The point isn't compared, as it can be anywhere along a side.
impl Close for auburn2d::prelude::CastHit {
    fn close(&self, other: &Self) -> bool {
        self.time.close(&other.time) && (self.time == 0.0 || self.normal.close(&other.normal))
    }
}

/// One query for one kind of pair, timed for auburn, for parry and for the hand-written
/// formula if there is one.
///
/// First the formula and parry have to agree on every pair, so that the benchmark times what
/// it says. Where auburn's answers differ from theirs, it says so, without stopping: finding
/// out how wrong auburn is is the randomized suites' job.
pub fn compare<R: Close + Debug>(
    c: &mut Criterion,
    id: &str,
    auburn: impl Fn(usize) -> R,
    parry: impl Fn(usize) -> R,
    formula: Option<impl Fn(usize) -> R>,
) {
    let mut differ = (0..PAIRS).filter_map(|i| {
        let expected = match &formula {
            Some(formula) => {
                let (expected, theirs) = (formula(i), parry(i));
                assert!(
                    expected.close(&theirs),
                    "{id}, pair {i}: the formula gives {expected:?}, parry {theirs:?}"
                );
                expected
            }
            None => parry(i),
        };
        let ours = auburn(i);
        (!ours.close(&expected)).then(|| format!("pair {i}: {ours:?}, not {expected:?}"))
    });
    if let Some(first) = differ.next() {
        let count = 1 + differ.count();
        let reference = if formula.is_some() {
            "the formula"
        } else {
            "parry"
        };
        println!(
            "{id}: auburn differs from {reference} on {count} of {PAIRS} pairs, such as {first}"
        );
    }

    let mut group = c.benchmark_group(id);
    group.bench_function("auburn", |b| run(b, &auburn));
    group.bench_function("parry", |b| run(b, &parry));
    if let Some(formula) = &formula {
        group.bench_function("formula", |b| run(b, formula));
    }
    group.finish();
}

/// Times `query` over the pairs in turn, one pair per iteration.
fn run<R>(b: &mut Bencher<'_>, query: impl Fn(usize) -> R) {
    let mut i = 0;
    b.iter(|| {
        i = (i + 1) % PAIRS;
        query(black_box(i))
    });
}

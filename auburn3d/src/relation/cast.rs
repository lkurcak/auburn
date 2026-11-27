//! Casts: when two shapes touch while one of them moves.
//!
//! `a.toi(&b, vel)` moves `a` at `vel`, its velocity relative to `b`, and answers in units of
//! time: at time `t`, `a` has moved by `t * vel`. Touching counts as overlapping.
//!
//! - [`TimeOfImpact::toi`] gives the first time from now, `t >= 0`, at which the shapes touch:
//!   0 if they overlap already.
//! - [`TimeOfImpactAndExit::toiae`] gives that time and the time they stop overlapping.
//! - [`TimeTravelingTimeOfImpact::tttoi`] and [`TimeTravelingTimeOfImpactAndExit::tttoiae`] give
//!   the same over all of time, as if `a` had always moved at `vel`, so their times can be
//!   negative: shapes that overlap now entered each other in the past.
//!
//! Each gives `None` if the shapes never touch: from now on for `toi` and `toiae`, and ever for
//! the time-traveling ones.
//!
//! # At rest
//!
//! A cast at zero velocity leaves the shapes where they are. If they overlap, they overlap
//! forever: `toi` gives 0, `toiae` gives (0, ∞), `tttoi` gives −∞ and `tttoiae` gives (−∞, ∞).
//! If they don't, all four give `None`. A velocity counts as zero when every component is too
//! small for its reciprocal to be finite: zero of either sign, or below about 3e-39.
//!
//! # Points
//!
//! Two points touch only where they coincide, as with
//! [`Collides`](crate::relation::collides::Collides). A point that moves through another one
//! enters and exits it at the same time, and one that misses it, by more than rounding, gives
//! `None`.

use crate::Vec3;

pub trait TimeOfImpact<T> {
    /// The first time from now (`t >= 0`) at which the shapes touch, with `self` moving at
    /// `vel`: 0 if they overlap already, `None` if never.
    ///
    /// `self` has moved by `t * vel` by then. See [the module](self) for the conventions,
    /// and [`TimeTravelingTimeOfImpact::tttoi`] for times in the past.
    fn toi(&self, other: &T, vel: Vec3) -> Option<f32>;
}

pub trait TimeTravelingTimeOfImpact<T> {
    /// The time at which the shapes start to touch, with `self` moving at `vel` all along:
    /// negative if that was in the past, `None` if never.
    ///
    /// See [the module](self) for the conventions, and [`TimeOfImpact::toi`] for the first
    /// contact from now on.
    fn tttoi(&self, other: &T, vel: Vec3) -> Option<f32>;
}

pub trait TimeOfImpactAndExit<T> {
    /// The times from now (`t >= 0`) at which the shapes start and stop overlapping, with
    /// `self` moving at `vel`: an entry of 0 if they overlap already, `None` if they never
    /// overlap from now on.
    ///
    /// See [the module](self) for the conventions.
    fn toiae(&self, other: &T, vel: Vec3) -> Option<(f32, f32)>;
}

pub trait TimeTravelingTimeOfImpactAndExit<T> {
    /// The times at which the shapes start and stop overlapping, with `self` moving at `vel`
    /// all along: negative if in the past, `None` if never.
    ///
    /// See [the module](self) for the conventions.
    fn tttoiae(&self, other: &T, vel: Vec3) -> Option<(f32, f32)>;
}

/// Whether a cast at `vel` counts as at rest: every component is too small for its reciprocal
/// to be finite (zero of either sign, or below about 3e-39).
pub(crate) fn is_at_rest(vel: Vec3) -> bool {
    !(1.0 / vel.abs().max_element()).is_finite()
}

/// The direction of a `vel` that is not at rest, and its speed. `vel` is scaled to a largest
/// component of 1 first, so that its squared length can't underflow.
pub(crate) fn direction_and_speed(vel: Vec3) -> (Vec3, f32) {
    let largest = vel.abs().max_element();
    let v = vel / largest;
    let length = v.length();
    (v / length, length * largest)
}

/// The four casts' answers from `span`, the times between which the shapes overlap over all of
/// time (`None` if never).
pub(crate) mod from_span {
    pub fn toi(span: Option<(f32, f32)>) -> Option<f32> {
        toiae(span).map(|(entry, _)| entry)
    }

    pub fn toiae(span: Option<(f32, f32)>) -> Option<(f32, f32)> {
        span.filter(|&(_, exit)| exit >= 0.0)
            .map(|(entry, exit)| (entry.max(0.0), exit))
    }

    pub fn tttoi(span: Option<(f32, f32)>) -> Option<f32> {
        span.map(|(entry, _)| entry)
    }

    pub fn tttoiae(span: Option<(f32, f32)>) -> Option<(f32, f32)> {
        span
    }
}

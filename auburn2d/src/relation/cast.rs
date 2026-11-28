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
//! [`Cast::cast`] gives the same time as `toi`, with how the shapes touch then: the contact's
//! normal, and a point where they touch. See [`CastHit`].
//!
//! # At rest
//!
//! A cast at zero velocity leaves the shapes where they are. If they overlap, they overlap
//! forever: `toi` gives 0, `toiae` gives (0, ∞), `tttoi` gives −∞ and `tttoiae` gives (−∞, ∞),
//! and `cast` gives a hit at 0. If they don't, all five give `None`. A velocity counts as zero when every component is too
//! small for its reciprocal to be finite: zero of either sign, or below about 3e-39.
//!
//! # Points
//!
//! Two points touch only where they coincide, as with
//! [`Collides`](crate::relation::collides::Collides). A point that moves through another one
//! enters and exits it at the same time, and one that misses it, by more than rounding, gives
//! `None`.
//!
//! # Contact normals
//!
//! Where shapes touch at a corner, more than one normal fits, and a cast's normal is one of
//! them. Where every direction fits, as for two points, it is against `vel`, or +x at rest.

use crate::Vec2;

/// Where and when a cast first touches: see [`Cast::cast`].
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CastHit {
    /// When the shapes first touch, from now on (`t >= 0`): 0 if they overlap already, as
    /// [`TimeOfImpact::toi`] gives it.
    pub time: f32,
    /// The contact's normal at `time`: a unit vector out of the other shape, towards the
    /// moving one, so moving that one along it parts them.
    ///
    /// If they overlap already, it is the way that parts them by the shortest move, as
    /// [`Penetrates`](crate::relation::penetrates::Penetrates) gives it, and for shapes that
    /// only touch, the normal where they touch.
    pub normal: Vec2,
    /// A point where the shapes touch at `time`, in world space.
    ///
    /// If they overlap already, where they would touch once the moving shape had moved out
    /// along `normal`: a point on the other shape's boundary.
    pub point: Vec2,
}

pub trait Cast<T> {
    /// When and how the shapes first touch from now on, with `self` moving at `vel`, or
    /// `None` if they never do.
    ///
    /// The time is [`TimeOfImpact::toi`]'s: 0 if they overlap already. See [`CastHit`] for the
    /// normal and the point, and [the module](self) for the conventions.
    fn cast(&self, other: &T, vel: Vec2) -> Option<CastHit>;
}

pub trait TimeOfImpact<T> {
    /// The first time from now (`t >= 0`) at which the shapes touch, with `self` moving at
    /// `vel`: 0 if they overlap already, `None` if never.
    ///
    /// `self` has moved by `t * vel` by then. See [the module](self) for the conventions,
    /// and [`TimeTravelingTimeOfImpact::tttoi`] for times in the past.
    fn toi(&self, other: &T, vel: Vec2) -> Option<f32>;
}

pub trait TimeTravelingTimeOfImpact<T> {
    /// The time at which the shapes start to touch, with `self` moving at `vel` all along:
    /// negative if that was in the past, `None` if never.
    ///
    /// See [the module](self) for the conventions, and [`TimeOfImpact::toi`] for the first
    /// contact from now on.
    fn tttoi(&self, other: &T, vel: Vec2) -> Option<f32>;
}

pub trait TimeOfImpactAndExit<T> {
    /// The times from now (`t >= 0`) at which the shapes start and stop overlapping, with
    /// `self` moving at `vel`: an entry of 0 if they overlap already, `None` if they never
    /// overlap from now on.
    ///
    /// See [the module](self) for the conventions.
    fn toiae(&self, other: &T, vel: Vec2) -> Option<(f32, f32)>;
}

pub trait TimeTravelingTimeOfImpactAndExit<T> {
    /// The times at which the shapes start and stop overlapping, with `self` moving at `vel`
    /// all along: negative if in the past, `None` if never.
    ///
    /// See [the module](self) for the conventions.
    fn tttoiae(&self, other: &T, vel: Vec2) -> Option<(f32, f32)>;
}

/// Whether a cast at `vel` counts as at rest: every component is too small for its reciprocal
/// to be finite (zero of either sign, or below about 3e-39).
#[inline]
pub(crate) fn is_at_rest(vel: Vec2) -> bool {
    // A compare, which costs less than the division: 2⁻¹²⁸ is the largest such, as its
    // reciprocal, 2¹²⁸, is past `f32::MAX`. NaN too, as the reciprocal's test has it.
    let largest = vel.abs().max_element();
    largest <= f32::from_bits(0x0020_0000) || largest.is_nan()
}

/// The direction of a `vel` that is not at rest, and its speed. `vel` is scaled to a largest
/// component of 1 first, so that its squared length can't underflow.
#[inline]
pub(crate) fn direction_and_speed(vel: Vec2) -> (Vec2, f32) {
    let largest = vel.abs().max_element();
    let v = vel / largest;
    let length = v.length();
    (v / length, length * largest)
}

/// `v` scaled to length 1, or `None` if it is too short to have a direction. It is scaled to a
/// largest component of 1 first, so that its squared length can't underflow.
#[inline]
pub(crate) fn unit(v: Vec2) -> Option<Vec2> {
    let largest = v.abs().max_element();
    if !(1.0 / largest).is_finite() {
        return None;
    }
    let v = v / largest;
    Some(v / v.length())
}

/// The normal where every direction fits: against `vel`, or +x at rest.
#[inline]
pub(crate) fn against(vel: Vec2) -> Vec2 {
    unit(-vel).unwrap_or(Vec2::X)
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

#[cfg(test)]
mod tests {
    use super::is_at_rest;
    use crate::Vec2;

    /// The compare agrees with the definition, that the reciprocal isn't finite, on either sign
    /// of the subnormals up to twice 2⁻¹²⁸ and of some larger numbers.
    #[test]
    fn at_rest_is_where_the_reciprocal_is_not_finite() {
        let larger = [f32::MIN_POSITIVE, 1e-30, 1.0, f32::MAX, f32::INFINITY];
        let small = (0..0x0040_0000_u32).map(f32::from_bits);
        for x in small.chain(larger) {
            for x in [x, -x] {
                let reciprocal_is_finite = (1.0 / x).is_finite();
                assert_eq!(
                    is_at_rest(Vec2::new(x, 0.0)),
                    !reciprocal_is_finite,
                    "{x:e}"
                );
                assert_eq!(
                    is_at_rest(Vec2::new(0.0, x)),
                    !reciprocal_is_finite,
                    "{x:e}"
                );
            }
        }
    }
}

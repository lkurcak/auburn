use crate::Vec2;
use crate::prelude::Box2d;
use crate::property::support::Support;
use std::f32::consts::TAU;
use std::sync::Arc;

use super::Shape;

#[derive(Debug, Clone)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// A convex polygon, given by its vertices in its local frame.
///
/// The vertices go around the polygon in order, either way round. The points are stored in an
/// `Arc` to allow efficient cloning.
///
/// With no points, as from [`Default`], the polygon is a single point at its origin.
///
/// # Examples
///
/// ```
/// use auburn2d::prelude::*;
/// use auburn2d::Vec2;
/// use std::sync::Arc;
///
/// let points = Arc::new(vec![
///     Vec2::new(-1.0, -1.0),
///     Vec2::new(1.0, -1.0),
///     Vec2::new(1.0, 1.0),
///     Vec2::new(-1.0, 1.0),
/// ]);
/// let poly2d = Poly2d::new(points);
/// assert_eq!(poly2d.points.len(), 4);
/// assert_eq!(poly2d.area(), 4.0);
/// ```
pub struct Poly2d {
    pub points: Arc<Vec<Vec2>>,
}

impl Default for Poly2d {
    fn default() -> Self {
        Self::new(Arc::new(Vec::new()))
    }
}

impl From<Box2d> for Poly2d {
    fn from(box2d: Box2d) -> Self {
        let points = Arc::new(vec![
            box2d.top_right(),
            box2d.top_left(),
            box2d.bottom_left(),
            box2d.bottom_right(),
        ]);
        Self::new(points)
    }
}

impl Poly2d {
    /// Creates a new `Poly2d` from its vertices, in order around it, either way round.
    ///
    /// The polygon has to be convex. Debug builds check that it is, allowing for rounding.
    pub fn new(points: Arc<Vec<Vec2>>) -> Self {
        debug_assert!(
            goes_around_convex_polygon(&points),
            "Poly2d::new: the points don't go once around a convex polygon: {points:?}"
        );
        Self { points }
    }

    /// Creates a regular `Poly2d` polygon with the given number of sides and circumradius and a
    /// "pointy top".
    ///
    /// The polygon is centered at the origin with vertices lying on a circle of the specified
    /// radius, counter-clockwise from the top.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    ///
    /// let hexagon = Poly2d::regular(6, 1.0);
    /// assert_eq!(hexagon.points.len(), 6);
    /// assert_eq!(hexagon.points[0], Vec2::new(0.0, 1.0));
    /// ```
    pub fn regular(sides: u32, radius: f32) -> Self {
        let points = Arc::new(
            (0..sides)
                .map(|i| {
                    let a = TAU * (i as f32) / (sides as f32);
                    radius * Vec2::new(-a.sin(), a.cos())
                })
                .collect(),
        );

        Self::new(points)
    }

    /// Creates a `Collider` with this shape at a given position (isometry).
    pub fn at<'a, I: crate::transformation::Transformation2d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }

    /// The vertices that the queries use: the points, or the origin if there are none.
    pub(crate) fn vertices(&self) -> &[Vec2] {
        if self.points.is_empty() {
            &[Vec2::ZERO]
        } else {
            &self.points
        }
    }
}

/// Whether `points` go once around a convex polygon, either way round, allowing for rounding.
/// Fewer than three points count, and so do points on a line.
fn goes_around_convex_polygon(points: &[Vec2]) -> bool {
    let n = points.len();
    if n < 3 {
        return true;
    }
    let (min, max) = points.iter().fold((points[0], points[0]), |(lo, hi), &p| {
        (lo.min(p), hi.max(p))
    });
    let size = (max - min).max_element();
    // A turn this sharp could go either way after rounding the points.
    let tolerance = 16.0 * f32::EPSILON * size * size;
    let edge = |i: usize| points[(i + 1) % n] - points[i % n];
    let (mut left, mut right, mut turned) = (false, false, 0.0f32);
    for i in 0..n {
        let (e, f) = (edge(i), edge(i + 1));
        let turn = e.perp_dot(f);
        left |= turn > tolerance;
        right |= turn < -tolerance;
        turned += turn.atan2(e.dot(f));
    }
    // A star turns the same way at every vertex too, but goes around more than once.
    !(left && right) && turned.abs() < 1.5 * TAU
}

impl Shape for Poly2d {
    fn area(&self) -> f32 {
        // The shoelace formula, about the first vertex so that the products stay small.
        let Some(&first) = self.points.first() else {
            return 0.0;
        };
        let twice: f32 = self
            .points
            .windows(2)
            .map(|w| (w[0] - first).perp_dot(w[1] - first))
            .sum();
        0.5 * twice.abs()
    }

    fn primitive(&self) -> super::Primitive<'_> {
        super::Primitive::Poly2d(self)
    }
}

impl Support for Poly2d {
    fn support(&self, dir: Vec2) -> Vec2 {
        let vertices = self.vertices();
        let mut result = vertices[0];
        let mut best = result.dot(dir);

        for p in &vertices[1..] {
            let score = p.dot(dir);
            if score > best {
                best = score;
                result = *p;
            }
        }

        result
    }
}

use crate::Vec2;
use crate::prelude::Box2d;
use crate::property::support::Support;
use std::sync::Arc;

use super::Shape;

#[derive(Debug, Clone)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
/// A 2D polygon defined by its vertices.
///
/// The polygon is stored as a list of points in local coordinates.
/// The points are stored in an `Arc` to allow efficient cloning.
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
    /// Creates a new `Poly2d` from a list of points.
    ///
    /// The points should be provided in counter-clockwise order for proper
    /// collision detection.
    pub const fn new(points: Arc<Vec<Vec2>>) -> Self {
        Self { points }
    }

    /// Creates a regular `Poly2d` polygon with the given number of sides and circumradius and a
    /// "pointy top".
    ///
    /// The polygon is centered at the origin with vertices lying on a circle
    /// of the specified radius.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    ///
    /// let hexagon = Poly2d::regular(6, 1.0);
    /// assert_eq!(hexagon.points.len(), 6);
    /// ```
    pub fn regular(sides: u32, radius: f32) -> Self {
        let points = Arc::new(
            (0..sides)
                .map(|i| {
                    let a = 2.0 * std::f32::consts::PI * (i as f32) / (sides as f32);
                    let x = a.sin();
                    let y = a.cos();
                    radius * Vec2::new(x, y)
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
}

impl Shape for Poly2d {
    fn area(&self) -> f32 {
        unimplemented!()
    }
}

impl Support for Poly2d {
    fn support(&self, dir: Vec2) -> Vec2 {
        let mut result = self.points[0];
        let mut best = result.dot(dir);

        for p in &self.points[1..] {
            let score = p.dot(dir);
            if score > best {
                best = score;
                result = *p;
            }
        }

        result
    }
}

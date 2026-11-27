use super::*;
use bevy::prelude::Transform;

/// Keeps the translation and the rotation about z.
///
/// [`Transform2d`] has one scale, so this is exact only when `scale.x == scale.y`. Otherwise
/// it keeps the larger of `|scale.x|` and `|scale.y|`, which still covers a [`Ball`] or a
/// [`Box2d`] but drops a mirror. The z translation, the z scale and any rotation off the z
/// axis are dropped.
///
/// [`Ball`]: crate::shape::ball::Ball
/// [`Box2d`]: crate::shape::box2d::Box2d
impl From<Transform> for Transform2d {
    fn from(transform: Transform) -> Self {
        let (x, y) = (transform.scale.x, transform.scale.y);
        Self {
            pos: transform.translation.truncate(),
            rot: Rotor2d::from_quaternion(transform.rotation),
            scale: if x == y { x } else { x.abs().max(y.abs()) },
        }
    }
}

/// A bevy [`Transform`] places a 2D shape in the plane `z = 0` and is seen down the z axis,
/// as a 2D camera shows it: its x and y scale stretch and mirror the shape, and its z
/// translation and z scale don't move it.
impl Transformation2d for Transform {
    fn apply_to_origin(&self) -> Vec2 {
        self.translation.truncate()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        self.transform_point(point.extend(0.0)).truncate()
    }

    /// The inverse in 3D. It is exact when it is itself a [`Transform`]: when the scale has
    /// the same magnitude on every axis, or the rotation is by quarter turns. It needs every
    /// scale nonzero, z included.
    fn inverse(&self) -> Self {
        Transform::from_matrix(self.to_matrix().inverse())
    }

    /// Bevy's `self * other`, which is exact when `self`'s scale is the same on every axis or
    /// `other` doesn't rotate.
    fn compose(&self, other: &Self) -> Self {
        *self * *other
    }

    /// Uses the real per-axis scale, like [`apply`](Transformation2d::apply).
    fn support_direction(&self, dir: Vec2) -> Vec2 {
        (self.scale * (self.rotation.inverse() * dir.extend(0.0))).truncate()
    }
}

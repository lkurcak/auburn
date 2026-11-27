use super::*;
use bevy::prelude::Transform;

impl Transformation3d for Transform {
    fn apply_to_origin(&self) -> Vec3 {
        self.translation
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.transform_point(point)
    }

    /// It is exact when the inverse is itself a [`Transform`]: when the scale has the same
    /// magnitude on every axis, or the rotation is by quarter turns.
    fn inverse(&self) -> Self {
        Transform::from_matrix(self.to_matrix().inverse())
    }

    /// Bevy's `self * other`, which is exact when `self`'s scale is the same on every axis or
    /// `other` doesn't rotate.
    fn compose(&self, other: &Self) -> Self {
        *self * *other
    }

    /// Uses the real per-axis scale, like [`apply`](Transformation3d::apply).
    fn support_direction(&self, dir: Vec3) -> Vec3 {
        self.scale * (self.rotation.inverse() * dir)
    }
}

impl From<Transform3d> for Transform {
    fn from(transform: Transform3d) -> Self {
        Transform {
            translation: transform.pos,
            rotation: transform.rot,
            scale: crate::Vec3::splat(transform.scale),
        }
    }
}

/// [`Transform3d`] has one scale, so this is exact only when the scale is the same on every
/// axis. Otherwise it keeps the largest magnitude, which still covers a [`Ball`] or a
/// [`Box3d`] but drops a mirror.
///
/// [`Ball`]: crate::shape::ball::Ball
/// [`Box3d`]: crate::shape::box3d::Box3d
impl From<Transform> for Transform3d {
    fn from(transform: Transform) -> Self {
        let s = transform.scale;
        Self {
            pos: transform.translation,
            rot: transform.rotation,
            scale: if s.x == s.y && s.y == s.z {
                s.x
            } else {
                s.abs().max_element()
            },
        }
    }
}

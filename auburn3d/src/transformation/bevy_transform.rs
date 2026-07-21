use super::*;

impl Transformation3d for bevy::prelude::Transform {
    fn apply_to_origin(&self) -> Vec3 {
        self.translation
    }

    fn apply(&self, point: Vec3) -> Vec3 {
        self.transform_point(point)
    }

    fn inverse(&self) -> Self {
        let t3d: Transform3d = (*self).into();
        let inv = t3d.inverse();
        inv.into()
    }

    fn compose(&self, other: &Self) -> Self {
        *self * *other
    }
}

impl From<Transform3d> for bevy::prelude::Transform {
    fn from(transform: Transform3d) -> Self {
        bevy::prelude::Transform {
            translation: transform.pos,
            rotation: transform.rot,
            scale: crate::Vec3::splat(transform.scale),
        }
    }
}

impl From<bevy::prelude::Transform> for Transform3d {
    fn from(transform: bevy::prelude::Transform) -> Self {
        let scale = transform.scale.max_element();
        Self {
            pos: transform.translation,
            rot: transform.rotation,
            scale,
        }
    }
}

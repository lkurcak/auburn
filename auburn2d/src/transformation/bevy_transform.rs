use super::*;

fn to_vec2(v: crate::Vec3) -> Vec2 {
    Vec2::new(v.x, v.y)
}

impl From<bevy::prelude::Transform> for Transform2d {
    fn from(transform: bevy::prelude::Transform) -> Self {
        Self {
            pos: to_vec2(transform.translation),
            rot: Rotor2d::from_quaternion(transform.rotation),
            scale: transform.scale.max_element(),
        }
    }
}

impl Transformation2d for bevy::prelude::Transform {
    fn apply_to_origin(&self) -> Vec2 {
        Into::<Transform2d>::into(*self).apply_to_origin()
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        Into::<Transform2d>::into(*self).apply(point)
    }

    fn inverse(&self) -> Self {
        let t2d: Transform2d = (*self).into();
        let inv = t2d.inverse();
        bevy::prelude::Transform {
            translation: crate::Vec3::new(inv.pos.x, inv.pos.y, self.translation.z),
            rotation: inv.rot.to_quaternion(),
            scale: crate::Vec3::splat(inv.scale),
        }
    }

    fn compose(&self, other: &Self) -> Self {
        let t2d: Transform2d = (*self).into();
        let other_t2d: Transform2d = (*other).into();
        let composed = t2d.compose(&other_t2d);
        bevy::prelude::Transform {
            translation: crate::Vec3::new(
                composed.pos.x,
                composed.pos.y,
                self.translation.z + other.translation.z,
            ),
            rotation: composed.rot.to_quaternion(),
            scale: crate::Vec3::splat(composed.scale),
        }
    }
}

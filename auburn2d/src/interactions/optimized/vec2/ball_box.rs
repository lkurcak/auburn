use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, box2d::Box2d, point::Point};
use crate::transformation::Transformation2d;

impl TimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Ball, Vec2> {
    fn toi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let flip_signs_based_on_vel = |mut v: Vec2| {
            if vel.x.is_sign_negative() {
                v.x = -v.x;
            }
            if vel.y.is_sign_negative() {
                v.y = -v.y;
            }
            v
        };

        let a = self.isometry.apply_to_origin();
        let b = other.isometry.apply_to_origin();
        let p = a - b;
        let inv_vel = 1.0 / vel;

        let mut x0 = (-p.x - other.shape.half_size.x - self.shape.radius) * inv_vel.x;
        let mut x1 = (-p.x - other.shape.half_size.x) * inv_vel.x;
        let mut x2 = (-p.x + other.shape.half_size.x) * inv_vel.x;
        let mut x3 = (-p.x + other.shape.half_size.x + self.shape.radius) * inv_vel.x;

        let mut y0 = (-p.y - other.shape.half_size.y - self.shape.radius) * inv_vel.y;
        let mut y1 = (-p.y - other.shape.half_size.y) * inv_vel.y;
        let mut y2 = (-p.y + other.shape.half_size.y) * inv_vel.y;
        let mut y3 = (-p.y + other.shape.half_size.y + self.shape.radius) * inv_vel.y;

        if vel.x.is_sign_negative() {
            std::mem::swap(&mut x1, &mut x2);
            std::mem::swap(&mut x0, &mut x3);
        }
        if vel.y.is_sign_negative() {
            std::mem::swap(&mut y1, &mut y2);
            std::mem::swap(&mut y0, &mut y3);
        }

        let point = Collider::new(&Point, &p);
        let ball = Ball::new(self.shape.radius);

        if x3.is_sign_negative() || y3.is_sign_negative() {
            return None;
        }

        if x1.is_sign_negative() && y1.is_sign_negative() {
            if x2.is_sign_negative() && y2.is_sign_negative() {
                let ball_center = flip_signs_based_on_vel(other.shape.top_right());
                let ball_collider = Collider::new(&ball, &ball_center);
                if point.collides(&ball_collider) {
                    return Some(0.0);
                } else {
                    return None;
                }
            } else {
                return Some(0.0);
            }
        }

        if x0.is_sign_negative() && y0.is_sign_negative() {
            if x1.is_sign_negative() {
                if x2.is_sign_negative() {
                    let ball_center = flip_signs_based_on_vel(other.shape.bottom_right());
                    return point.toi(&Collider::new(&ball, &ball_center), vel);
                } else {
                    return Some(0.0);
                }
            }

            if y1.is_sign_negative() {
                if y2.is_sign_negative() {
                    let ball_center = flip_signs_based_on_vel(other.shape.top_left());
                    return point.toi(&Collider::new(&ball, &ball_center), vel);
                } else {
                    return Some(0.0);
                }
            }

            let ball_center = flip_signs_based_on_vel(other.shape.bottom_left());
            return point.toi(&Collider::new(&ball, &ball_center), vel);
        }

        if x0 < y0 {
            if x1 < y0 {
                if x2 < y0 {
                    if x3 < y0 {
                        None
                    } else {
                        let ball_center = flip_signs_based_on_vel(other.shape.bottom_right());
                        point.toi(&Collider::new(&ball, &ball_center), vel)
                    }
                } else {
                    if x2.is_sign_negative() {
                        let ball_center = flip_signs_based_on_vel(other.shape.bottom_right());
                        if point.collides(&Collider::new(&ball, &ball_center)) {
                            Some(0.0)
                        } else {
                            None
                        }
                    } else {
                        Some(y0)
                    }
                }
            } else {
                let ball_center = flip_signs_based_on_vel(other.shape.bottom_left());
                point.toi(&Collider::new(&ball, &ball_center), vel)
            }
        } else {
            if y1 < x0 {
                if y2 < x0 {
                    if y3 < x0 {
                        None
                    } else {
                        let ball_center = flip_signs_based_on_vel(other.shape.top_left());
                        point.toi(&Collider::new(&ball, &ball_center), vel)
                    }
                } else {
                    if y2.is_sign_negative() {
                        let ball_center = flip_signs_based_on_vel(other.shape.top_left());
                        if point.collides(&Collider::new(&ball, &ball_center)) {
                            Some(0.0)
                        } else {
                            None
                        }
                    } else {
                        Some(x0)
                    }
                }
            } else {
                let ball_center = flip_signs_based_on_vel(other.shape.bottom_left());
                point.toi(&Collider::new(&ball, &ball_center), vel)
            }
        }
    }
}
impl TimeOfImpact<Collider<'_, Ball, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toi(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(other.shape, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.toi(&b, vel)
    }
}
impl TimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Ball, Vec2> {
    fn toiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        self.tttoiae(other, vel).and_then(|(toi, toe)| {
            if toe.is_sign_negative() {
                None
            } else {
                Some((toi.max(0.0), toe))
            }
        })
    }
}
impl TimeOfImpactAndExit<Collider<'_, Ball, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn toiae(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(other.shape, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.toiae(&b, vel)
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Box2d, Vec2>> for Collider<'_, Ball, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<f32> {
        let flip_signs_based_on_vel = |mut v: Vec2| {
            if vel.x.is_sign_negative() {
                v.x = -v.x;
            }
            if vel.y.is_sign_negative() {
                v.y = -v.y;
            }
            v
        };

        let a = self.isometry.apply_to_origin();
        let b = other.isometry.apply_to_origin();
        let p = a - b;
        let inv_vel = 1.0 / vel;

        let mut x0 = (-p.x - other.shape.half_size.x - self.shape.radius) * inv_vel.x;
        let mut x1 = (-p.x - other.shape.half_size.x) * inv_vel.x;
        let mut x2 = (-p.x + other.shape.half_size.x) * inv_vel.x;
        let mut x3 = (-p.x + other.shape.half_size.x + self.shape.radius) * inv_vel.x;

        let mut y0 = (-p.y - other.shape.half_size.y - self.shape.radius) * inv_vel.y;
        let mut y1 = (-p.y - other.shape.half_size.y) * inv_vel.y;
        let mut y2 = (-p.y + other.shape.half_size.y) * inv_vel.y;
        let mut y3 = (-p.y + other.shape.half_size.y + self.shape.radius) * inv_vel.y;

        if vel.x.is_sign_negative() {
            std::mem::swap(&mut x1, &mut x2);
            std::mem::swap(&mut x0, &mut x3);
        }
        if vel.y.is_sign_negative() {
            std::mem::swap(&mut y1, &mut y2);
            std::mem::swap(&mut y0, &mut y3);
        }

        let point = Collider::new(&Point, &p);
        let ball = Ball::new(self.shape.radius);

        if x0 < y0 {
            if x1 < y0 {
                if x2 < y0 {
                    if x3 < y0 {
                        None
                    } else {
                        let ball_center = flip_signs_based_on_vel(other.shape.bottom_right());
                        point.tttoi(&Collider::new(&ball, &ball_center), vel)
                    }
                } else {
                    Some(y0)
                }
            } else {
                let ball_center = flip_signs_based_on_vel(other.shape.bottom_left());
                point.tttoi(&Collider::new(&ball, &ball_center), vel)
            }
        } else if y1 < x0 {
            if y2 < x0 {
                if y3 < x0 {
                    None
                } else {
                    let ball_center = flip_signs_based_on_vel(other.shape.top_left());
                    point.tttoi(&Collider::new(&ball, &ball_center), vel)
                }
            } else {
                Some(x0)
            }
        } else {
            let ball_center = flip_signs_based_on_vel(other.shape.bottom_left());
            point.tttoi(&Collider::new(&ball, &ball_center), vel)
        }
    }
}
impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoi(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<f32> {
        let a = Collider::new(other.shape, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoi(&b, vel)
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box2d, Vec2>> for Collider<'_, Ball, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Box2d, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        if let Some(toi) = self.tttoi(other, vel) {
            let toe = self.tttoi(other, -vel).map(|x| -x).unwrap_or(toi);
            Some((toi, toe))
        } else {
            None
        }
    }
}
impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Vec2>> for Collider<'_, Box2d, Vec2> {
    fn tttoiae(&self, other: &Collider<'_, Ball, Vec2>, vel: Vec2) -> Option<(f32, f32)> {
        let a = Collider::new(other.shape, self.isometry);
        let b = Collider::new(self.shape, other.isometry);
        a.tttoiae(&b, vel)
    }
}

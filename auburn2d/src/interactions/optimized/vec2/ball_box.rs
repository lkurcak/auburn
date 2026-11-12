use crate::Vec2;
use crate::collider::Collider;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::{ball::Ball, box2d::Box2d, point::Point};
use crate::transformation::Transformation2d;

use super::box_point::slab;

/// Whether `vel` runs along an axis: one of its components is too small for its reciprocal to
/// be finite (zero of either sign, or subnormal). The slab method in the casts below divides by
/// the velocity, which then gives `0 * inf = NaN` wherever the ball's path lies exactly on one
/// of the slabs' edges, so these go to [`along_an_axis`] instead.
fn is_along_an_axis(vel: Vec2) -> bool {
    !(1.0 / vel.x).is_finite() || !(1.0 / vel.y).is_finite()
}

/// When a ball moving along an axis (see [`is_along_an_axis`]) overlaps a box, over all of
/// time (touching counts): from when to when, or `None` if never. `p` is the ball's center
/// relative to the box's. It takes the rounded box's cross-section along the ball's path.
fn along_an_axis(p: Vec2, half: Vec2, radius: f32, vel: Vec2) -> Option<(f32, f32)> {
    if (1.0 / vel.x).is_finite() {
        // Moving along x: the same with the axes swapped
        let swap = |v: Vec2| Vec2::new(v.y, v.x);
        return along_y(swap(p), swap(half), radius, vel.x);
    }
    along_y(p, half, radius, vel.y)
}

/// [`along_an_axis`] for a ball that keeps its x (and moves along y, if at all).
fn along_y(p: Vec2, half: Vec2, radius: f32, vel_y: f32) -> Option<(f32, f32)> {
    // How far above and below the box's center the ball touches it, at the ball's x
    let dx = p.x.abs() - half.x;
    let reach = if dx <= 0.0 {
        half.y + radius
    } else if dx <= radius {
        // Past the side, only a rounded corner is in the way
        half.y + (radius * radius - dx * dx).sqrt()
    } else {
        return None;
    };
    slab(-reach - p.y, reach - p.y, vel_y)
}

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
        if is_along_an_axis(vel) {
            return along_an_axis(p, other.shape.half_size, self.shape.radius, vel)
                .and_then(|(enter, exit)| (exit >= 0.0).then_some(enter.max(0.0)));
        }
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
        self.tttoiae(other, vel)
            .and_then(|(toi, toe)| (toe >= 0.0).then_some((toi.max(0.0), toe)))
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
        if is_along_an_axis(vel) {
            return along_an_axis(p, other.shape.half_size, self.shape.radius, vel)
                .map(|(enter, _)| enter);
        }
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
        if is_along_an_axis(vel) {
            let p = self.isometry.apply_to_origin() - other.isometry.apply_to_origin();
            return along_an_axis(p, other.shape.half_size, self.shape.radius, vel);
        }
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

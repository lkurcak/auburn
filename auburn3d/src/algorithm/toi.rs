use super::gjk::gjk;
/// Time of Impact (TOI) using Conservative Advancement algorithm.
use crate::Vec3;

const MAX_ITERATIONS: usize = 32;
const TOLERANCE: f32 = 1e-4;

/// Conservative Advancement algorithm for Time of Impact.
pub fn conservative_advancement_toi<F1, F2>(support_a: F1, support_b: F2, vel: Vec3) -> Option<f32>
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    if vel.length_squared() < 1e-10 {
        return gjk(support_a, support_b, 30).map(|_| 0.0);
    }

    let vel_length = vel.length();
    let vel_normalized = vel / vel_length;

    let mut t = 0.0;

    for _ in 0..MAX_ITERATIONS {
        let support_a_t = |dir: Vec3| support_a(dir) + vel * t;
        let support_b_t = |dir: Vec3| support_b(dir);

        if gjk(support_a_t, support_b_t, 30).is_some() {
            return Some(t);
        }

        let distance = estimate_distance(&support_a_t, &support_b_t, vel_normalized);
        let dt = ((distance * 0.9) / vel_length).min(1.0 - t);

        if dt < TOLERANCE {
            let support_a_end = |dir: Vec3| support_a(dir) + vel;
            if gjk(support_a_end, support_b, 30).is_some() {
                return Some(1.0);
            }
            return None;
        }

        t += dt;

        if t >= 1.0 - TOLERANCE {
            let support_a_end = |dir: Vec3| support_a(dir) + vel;
            if gjk(support_a_end, support_b, 30).is_some() {
                return Some(1.0);
            }
            return None;
        }
    }

    let support_a_end = |dir: Vec3| support_a(dir) + vel;
    if gjk(support_a_end, support_b, 30).is_some() {
        Some(1.0)
    } else {
        None
    }
}

fn estimate_distance<F1, F2>(support_a: &F1, support_b: &F2, direction: Vec3) -> f32
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    let point_a = support_a(direction);
    let point_b = support_b(-direction);
    let separation = (point_b - point_a).dot(direction);
    separation.max(0.0)
}

pub fn conservative_advancement_toiae<F1, F2>(
    support_a: F1,
    support_b: F2,
    vel: Vec3,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec3) -> Vec3 + Copy,
    F2: Fn(Vec3) -> Vec3 + Copy,
{
    let t_entry = conservative_advancement_toi(support_a, support_b, vel)?;

    let mut t_exit = t_entry;
    let dt = 0.01;

    for _ in 0..100 {
        t_exit += dt;
        if t_exit > 1.0 {
            t_exit = 1.0;
            break;
        }

        let support_a_t = |dir: Vec3| support_a(dir) + vel * t_exit;
        let support_b_t = |dir: Vec3| support_b(dir);

        if gjk(support_a_t, support_b_t, 30).is_none() {
            break;
        }
    }

    Some((t_entry, t_exit))
}

pub fn conservative_advancement_toi_unbounded<F1, F2>(
    support_a: F1,
    support_b: F2,
    vel: Vec3,
) -> Option<f32>
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    if vel.length_squared() < 1e-10 {
        return gjk(support_a, support_b, 30).map(|_| 0.0);
    }

    let vel_length = vel.length();
    let vel_normalized = vel / vel_length;

    let mut t = 0.0;

    for _ in 0..MAX_ITERATIONS {
        let support_a_t = |dir: Vec3| support_a(dir) + vel * t;
        let support_b_t = |dir: Vec3| support_b(dir);

        if gjk(support_a_t, support_b_t, 30).is_some() {
            return Some(t);
        }

        let distance = estimate_distance(&support_a_t, &support_b_t, vel_normalized);
        let dt = distance * 0.9 / vel_length;

        if dt < TOLERANCE {
            let t_final = t + TOLERANCE;
            if gjk(|dir| support_a(dir) + vel * t_final, support_b, 30).is_some() {
                return Some(t_final);
            }
            return None;
        }

        t += dt;
    }

    None
}

pub fn conservative_advancement_toiae_unbounded<F1, F2>(
    support_a: F1,
    support_b: F2,
    vel: Vec3,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec3) -> Vec3 + Copy,
    F2: Fn(Vec3) -> Vec3 + Copy,
{
    let t_entry = conservative_advancement_toi_unbounded(support_a, support_b, vel)?;

    let mut t_high = t_entry + 1.0;
    for _ in 0..20 {
        let support_a_t = |dir: Vec3| support_a(dir) + vel * t_high;
        let support_b_t = |dir: Vec3| support_b(dir);
        if gjk(support_a_t, support_b_t, 30).is_none() {
            break;
        }
        t_high *= 2.0;
    }

    let mut t_low = t_entry;
    let mut t_exit = t_high;
    for _ in 0..32 {
        let t_mid = (t_low + t_exit) / 2.0;
        let support_a_t = |dir: Vec3| support_a(dir) + vel * t_mid;
        let support_b_t = |dir: Vec3| support_b(dir);
        if gjk(support_a_t, support_b_t, 30).is_some() {
            t_low = t_mid;
        } else {
            t_exit = t_mid;
        }
    }

    Some((t_entry, t_exit))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_toi_spheres_will_collide() {
        let center_a = Vec3::ZERO;
        let radius_a = 1.0;
        let center_b = Vec3::new(3.0, 0.0, 0.0);
        let radius_b = 1.0;
        let vel = Vec3::new(1.0, 0.0, 0.0);

        let support_a = |dir: Vec3| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec3| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi(support_a, support_b, vel);

        assert!(toi.is_some());
        let t = toi.unwrap();
        assert!(t > 0.9 && t <= 1.0, "Expected t near 1.0, got {}", t);
    }

    #[test]
    fn test_toi_spheres_wont_collide() {
        let center_a = Vec3::ZERO;
        let radius_a = 1.0;
        let center_b = Vec3::new(3.0, 3.0, 0.0);
        let radius_b = 1.0;
        let vel = Vec3::new(1.0, 0.0, 0.0);

        let support_a = |dir: Vec3| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec3| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi(support_a, support_b, vel);
        assert!(toi.is_none());
    }

    #[test]
    fn test_toi_already_colliding() {
        let center_a = Vec3::ZERO;
        let radius_a = 1.0;
        let center_b = Vec3::new(0.5, 0.0, 0.0);
        let radius_b = 1.0;
        let vel = Vec3::new(1.0, 0.0, 0.0);

        let support_a = |dir: Vec3| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec3| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi(support_a, support_b, vel);
        assert_eq!(toi, Some(0.0));
    }
}

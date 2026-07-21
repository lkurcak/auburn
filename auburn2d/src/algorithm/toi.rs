use super::gjk::gjk;
/// Time of Impact (TOI) using Conservative Advancement algorithm.
/// This provides a generic fallback that works with any shapes that implement Support.
use crate::Vec2;

const MAX_ITERATIONS: usize = 32;
const TOLERANCE: f32 = 1e-4;

/// Conservative Advancement algorithm for Time of Impact.
///
/// Finds the time t ∈ [0, 1] when two shapes first collide given relative velocity.
///
/// # Arguments
/// * `support_a` - Support function for shape A at t=0
/// * `support_b` - Support function for shape B at t=0
/// * `vel` - Relative velocity (velocity of A relative to B)
///
/// # Returns
/// * `Some(t)` - Time of first impact in [0, 1]
/// * `None` - No collision in time interval
pub fn conservative_advancement_toi<F1, F2>(support_a: F1, support_b: F2, vel: Vec2) -> Option<f32>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    // If no relative velocity, just check current state
    if vel.length_squared() < 1e-10 {
        return gjk(support_a, support_b, 30).map(|_| 0.0);
    }

    let vel_length = vel.length();
    let vel_normalized = vel / vel_length;

    let mut t = 0.0;

    for _ in 0..MAX_ITERATIONS {
        // Create support functions at current time t
        let support_a_t = |dir: Vec2| support_a(dir) + vel * t;
        let support_b_t = |dir: Vec2| support_b(dir);

        // Check if colliding at time t
        if gjk(support_a_t, support_b_t, 30).is_some() {
            return Some(t);
        }

        // Estimate distance between shapes using support functions
        // This is a conservative (safe) lower bound on the actual distance
        let distance = estimate_distance(&support_a_t, &support_b_t, vel_normalized);

        // Advance time conservatively
        // Use 90% of estimated time to be more conservative
        let dt = ((distance * 0.9) / vel_length).min(1.0 - t);

        if dt < TOLERANCE {
            // Check one more time at t=1.0 before giving up
            let support_a_end = |dir: Vec2| support_a(dir) + vel;
            if gjk(support_a_end, support_b, 30).is_some() {
                return Some(1.0);
            }
            return None;
        }

        t += dt;

        // If we've reached or passed t=1, check final position
        if t >= 1.0 - TOLERANCE {
            let support_a_end = |dir: Vec2| support_a(dir) + vel;
            if gjk(support_a_end, support_b, 30).is_some() {
                return Some(1.0);
            }
            return None;
        }
    }

    // Max iterations reached, check final position as last resort
    let support_a_end = |dir: Vec2| support_a(dir) + vel;
    if gjk(support_a_end, support_b, 30).is_some() {
        Some(1.0)
    } else {
        None
    }
}

/// Estimate the distance between two shapes using support functions.
/// This provides a conservative lower bound using the support in the direction of motion.
fn estimate_distance<F1, F2>(support_a: &F1, support_b: &F2, direction: Vec2) -> f32
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    // Get the closest points in the direction of relative motion
    let point_a = support_a(direction);
    let point_b = support_b(-direction);

    // Distance along the direction of motion
    let separation = (point_b - point_a).dot(direction);

    // Return conservative estimate (always positive)
    separation.max(0.0)
}

/// Time of Impact with entry and exit times.
/// Returns both when shapes start and stop overlapping.
pub fn conservative_advancement_toiae<F1, F2>(
    support_a: F1,
    support_b: F2,
    vel: Vec2,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec2) -> Vec2 + Copy,
    F2: Fn(Vec2) -> Vec2 + Copy,
{
    // Find entry time
    let t_entry = conservative_advancement_toi(support_a, support_b, vel)?;

    // Find exit time by searching forward from entry
    let mut t_exit = t_entry;
    let dt = 0.01; // Step size for finding exit

    for _ in 0..100 {
        t_exit += dt;
        if t_exit > 1.0 {
            t_exit = 1.0;
            break;
        }

        let support_a_t = |dir: Vec2| support_a(dir) + vel * t_exit;
        let support_b_t = |dir: Vec2| support_b(dir);

        // If no longer colliding, we found the exit time
        if gjk(support_a_t, support_b_t, 30).is_none() {
            break;
        }
    }

    Some((t_entry, t_exit))
}

/// Unbounded version of conservative_advancement_toi that finds TOI without a time limit.
///
/// Works for any shapes that implement Support, at any time scale.
pub fn conservative_advancement_toi_unbounded<F1, F2>(
    support_a: F1,
    support_b: F2,
    vel: Vec2,
) -> Option<f32>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    if vel.length_squared() < 1e-10 {
        return gjk(support_a, support_b, 30).map(|_| 0.0);
    }

    let vel_length = vel.length();
    let vel_normalized = vel / vel_length;

    let mut t = 0.0;

    for _ in 0..MAX_ITERATIONS {
        let support_a_t = |dir: Vec2| support_a(dir) + vel * t;
        let support_b_t = |dir: Vec2| support_b(dir);

        if gjk(support_a_t, support_b_t, 30).is_some() {
            return Some(t);
        }

        let distance = estimate_distance(&support_a_t, &support_b_t, vel_normalized);
        let dt = distance * 0.9 / vel_length;

        if dt < TOLERANCE {
            // Check one more time at t + TOLERANCE before giving up
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

/// Unbounded version of conservative_advancement_toiae that finds TOI and exit time without a time limit.
pub fn conservative_advancement_toiae_unbounded<F1, F2>(
    support_a: F1,
    support_b: F2,
    vel: Vec2,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec2) -> Vec2 + Copy,
    F2: Fn(Vec2) -> Vec2 + Copy,
{
    let t_entry = conservative_advancement_toi_unbounded(support_a, support_b, vel)?;

    // Find an upper bound for the exit time by doubling
    let mut t_high = t_entry + 1.0;
    for _ in 0..20 {
        let support_a_t = |dir: Vec2| support_a(dir) + vel * t_high;
        let support_b_t = |dir: Vec2| support_b(dir);
        if gjk(support_a_t, support_b_t, 30).is_none() {
            break;
        }
        t_high *= 2.0;
    }

    // Binary search for the exact exit time
    let mut t_low = t_entry;
    let mut t_exit = t_high;
    for _ in 0..32 {
        let t_mid = (t_low + t_exit) / 2.0;
        let support_a_t = |dir: Vec2| support_a(dir) + vel * t_mid;
        let support_b_t = |dir: Vec2| support_b(dir);
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
    fn test_toi_circles_will_collide() {
        // Ball A at origin, radius 1
        let center_a = Vec2::ZERO;
        let radius_a = 1.0;

        // Ball B at (3, 0), radius 1
        let center_b = Vec2::new(3.0, 0.0);
        let radius_b = 1.0;

        // A moves right with velocity 1, should hit at t = 1.0
        let vel = Vec2::new(1.0, 0.0);

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi(support_a, support_b, vel);

        assert!(toi.is_some());
        let t = toi.unwrap();
        assert!(t > 0.9 && t <= 1.0, "Expected t near 1.0, got {}", t);
    }

    #[test]
    fn test_toi_circles_wont_collide() {
        // Ball A at origin, radius 1
        let center_a = Vec2::ZERO;
        let radius_a = 1.0;

        // Ball B at (3, 3), radius 1 - too far away
        let center_b = Vec2::new(3.0, 3.0);
        let radius_b = 1.0;

        // A moves right, will miss B
        let vel = Vec2::new(1.0, 0.0);

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi(support_a, support_b, vel);

        assert!(toi.is_none());
    }

    #[test]
    fn test_toi_already_colliding() {
        // Overlapping balls
        let center_a = Vec2::ZERO;
        let radius_a = 1.0;

        let center_b = Vec2::new(0.5, 0.0);
        let radius_b = 1.0;

        let vel = Vec2::new(1.0, 0.0);

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi(support_a, support_b, vel);

        assert_eq!(toi, Some(0.0));
    }

    #[test]
    fn test_toi_unbounded_circles_far_collision() {
        // Ball A at origin, radius 1
        let center_a = Vec2::ZERO;
        let radius_a = 1.0;

        // Ball B at (10, 0), radius 1
        let center_b = Vec2::new(10.0, 0.0);
        let radius_b = 1.0;

        // A moves right with velocity 1, should hit at t = 8.0
        let vel = Vec2::new(1.0, 0.0);

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let toi = conservative_advancement_toi_unbounded(support_a, support_b, vel);

        assert!(toi.is_some());
        let t = toi.unwrap();
        assert!(t > 7.9 && t <= 8.2, "Expected t near 8.0, got {}", t);
    }
}

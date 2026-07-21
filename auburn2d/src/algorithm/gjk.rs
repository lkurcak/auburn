/// Gilbert-Johnson-Keerthi (GJK) algorithm for collision detection.
/// This is a general-purpose collision detection algorithm that works with any convex shape
/// that implements the Support trait.
use crate::Vec2;

/// A simplex is a set of up to 3 points that we use to determine if the origin is contained
/// in the Minkowski difference of two shapes.
#[derive(Debug, Clone)]
pub struct Simplex {
    points: [Vec2; 3],
    count: usize,
}

impl Simplex {
    fn new() -> Self {
        Self {
            points: [Vec2::ZERO; 3],
            count: 0,
        }
    }

    fn push(&mut self, point: Vec2) {
        self.points[self.count] = point;
        self.count += 1;
    }

    pub fn get(&self, index: usize) -> Vec2 {
        self.points[index]
    }

    pub fn count(&self) -> usize {
        self.count
    }

    fn set(&mut self, points: &[Vec2]) {
        self.count = points.len();
        for (i, &point) in points.iter().enumerate() {
            self.points[i] = point;
        }
    }
}

/// Check if the simplex contains the origin and update the search direction.
/// Returns true if the origin is contained in the simplex (collision detected).
fn handle_simplex(simplex: &mut Simplex, direction: &mut Vec2) -> bool {
    match simplex.count() {
        2 => handle_line_simplex(simplex, direction),
        3 => handle_triangle_simplex(simplex, direction),
        _ => false,
    }
}

/// Handle the case where the simplex is a line segment.
fn handle_line_simplex(simplex: &mut Simplex, direction: &mut Vec2) -> bool {
    let a = simplex.get(1);
    let b = simplex.get(0);

    let ab = b - a;
    let ao = -a;

    // Check if the origin is in the direction of b
    if ab.dot(ao) > 0.0 {
        // The origin is somewhere past b in the direction perpendicular to ab
        *direction = triple_product(ab, ao, ab);
        if direction.length_squared() < 1e-10 {
            // ab and ao are parallel, use perpendicular
            *direction = Vec2::new(-ab.y, ab.x);
        }
    } else {
        // The origin is closest to a
        simplex.set(&[a]);
        *direction = ao;
    }

    false
}

/// Handle the case where the simplex is a triangle.
fn handle_triangle_simplex(simplex: &mut Simplex, direction: &mut Vec2) -> bool {
    let a = simplex.get(2);
    let b = simplex.get(1);
    let c = simplex.get(0);

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let ab_perp = triple_product(ac, ab, ab);
    let ac_perp = triple_product(ab, ac, ac);

    if ab_perp.dot(ao) > 0.0 {
        // Origin is closest to edge ab
        simplex.set(&[b, a]);
        *direction = ab_perp;
    } else if ac_perp.dot(ao) > 0.0 {
        // Origin is closest to edge ac
        simplex.set(&[c, a]);
        *direction = ac_perp;
    } else {
        // Origin is inside the triangle
        return true;
    }

    false
}

/// Compute the triple product: (a × b) × c in 2D
/// This gives us a vector perpendicular to c in the plane, pointing away from a
fn triple_product(a: Vec2, b: Vec2, c: Vec2) -> Vec2 {
    // In 2D: (a × b) × c = b(c·a) - a(c·b)
    // where × is the cross product (scalar in 2D)
    let cross_ab = a.x * b.y - a.y * b.x;
    Vec2::new(-c.y * cross_ab, c.x * cross_ab)
}

/// Run the GJK algorithm to detect collision between two shapes.
///
/// # Arguments
/// * `support_a` - Support function for shape A
/// * `support_b` - Support function for shape B
/// * `max_iterations` - Maximum number of iterations (typically 20-30)
///
/// # Returns
/// * `Some(simplex)` if collision is detected, containing the final simplex
/// * `None` if no collision
pub fn gjk<F1, F2>(support_a: F1, support_b: F2, max_iterations: usize) -> Option<Simplex>
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    // Minkowski difference support function
    let support = |dir: Vec2| -> Vec2 { support_a(dir) - support_b(-dir) };

    let mut simplex = Simplex::new();
    let mut direction = Vec2::new(1.0, 0.0);

    // Get the first point
    let first_point = support(direction);
    simplex.push(first_point);
    direction = -first_point;

    for _ in 0..max_iterations {
        let a = support(direction);

        // If we didn't pass the origin, there's no collision
        if a.dot(direction) < 0.0 {
            return None;
        }

        simplex.push(a);

        if handle_simplex(&mut simplex, &mut direction) {
            return Some(simplex);
        }
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn test_gjk_circles_colliding() {
        // Two circles at distance 2, each with radius 1.5 -> should collide
        let center_a = Vec2::new(0.0, 0.0);
        let radius_a = 1.5;
        let center_b = Vec2::new(2.0, 0.0);
        let radius_b = 1.5;

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let result = gjk(support_a, support_b, 30);
        assert!(result.is_some());
    }

    #[test]
    fn test_gjk_circles_not_colliding() {
        // Two circles at distance 4, each with radius 1.5 -> should not collide
        let center_a = Vec2::new(0.0, 0.0);
        let radius_a = 1.5;
        let center_b = Vec2::new(4.0, 0.0);
        let radius_b = 1.5;

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let result = gjk(support_a, support_b, 30);
        assert!(result.is_none());
    }
}

/// Gilbert-Johnson-Keerthi (GJK) algorithm for collision detection.
/// This is a general-purpose collision detection algorithm that works with any convex shape
/// that implements the Support trait.
use crate::Vec3;

/// A simplex is a set of up to 4 points that we use to determine if the origin is contained
/// in the Minkowski difference of two shapes.
#[derive(Debug, Clone)]
pub struct Simplex {
    points: [Vec3; 4],
    count: usize,
}

impl Simplex {
    fn new() -> Self {
        Self {
            points: [Vec3::ZERO; 4],
            count: 0,
        }
    }

    fn push(&mut self, point: Vec3) {
        self.points[self.count] = point;
        self.count += 1;
    }

    pub fn get(&self, index: usize) -> Vec3 {
        self.points[index]
    }

    pub fn count(&self) -> usize {
        self.count
    }

    fn set(&mut self, points: &[Vec3]) {
        self.count = points.len();
        for (i, &point) in points.iter().enumerate() {
            self.points[i] = point;
        }
    }
}

/// Check if the simplex contains the origin and update the search direction.
/// Returns true if the origin is contained in the simplex (collision detected).
fn handle_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    match simplex.count() {
        2 => handle_line_simplex(simplex, direction),
        3 => handle_triangle_simplex(simplex, direction),
        4 => handle_tetrahedron_simplex(simplex, direction),
        _ => false,
    }
}

/// Handle the case where the simplex is a line segment.
fn handle_line_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let a = simplex.get(1);
    let b = simplex.get(0);

    let ab = b - a;
    let ao = -a;

    if ab.dot(ao) > 0.0 {
        // The origin is somewhere past b in the direction perpendicular to ab
        *direction = ab.cross(ao).cross(ab);
        if direction.length_squared() < 1e-10 {
            // ab and ao are parallel, use perpendicular
            *direction = ab.any_orthonormal_vector();
        }
    } else {
        // The origin is closest to a
        simplex.set(&[a]);
        *direction = ao;
    }

    false
}

/// Handle the case where the simplex is a triangle.
fn handle_triangle_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let a = simplex.get(2);
    let b = simplex.get(1);
    let c = simplex.get(0);

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let abc = ab.cross(ac);

    // Edge AB: perpendicular pointing away from C
    let ab_perp = ab.cross(abc);
    if ab_perp.dot(ao) > 0.0 {
        simplex.set(&[b, a]);
        *direction = ab_perp;
        return false;
    }

    // Edge AC: perpendicular pointing away from B
    let ac_perp = abc.cross(ac);
    if ac_perp.dot(ao) > 0.0 {
        simplex.set(&[c, a]);
        *direction = ac_perp;
        return false;
    }

    // Origin is in the triangle plane; check which side it is on
    if abc.dot(ao) > 0.0 {
        // Above the triangle
        *direction = abc;
    } else {
        // Below the triangle; swap b and c to keep winding consistent
        simplex.set(&[b, c, a]);
        *direction = -abc;
    }

    false
}

/// Handle the case where the simplex is a tetrahedron.
fn handle_tetrahedron_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let a = simplex.get(3);
    let b = simplex.get(2);
    let c = simplex.get(1);
    let d = simplex.get(0);

    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let ao = -a;

    // Face ABC: opposite vertex D
    let abc = ab.cross(ac);
    let abc_normal = if abc.dot(ad) > 0.0 { -abc } else { abc };
    if abc_normal.dot(ao) > 0.0 {
        simplex.set(&[c, b, a]);
        *direction = abc_normal;
        return handle_triangle_simplex(simplex, direction);
    }

    // Face ABD: opposite vertex C
    let abd = ab.cross(ad);
    let abd_normal = if abd.dot(ac) > 0.0 { -abd } else { abd };
    if abd_normal.dot(ao) > 0.0 {
        simplex.set(&[d, b, a]);
        *direction = abd_normal;
        return handle_triangle_simplex(simplex, direction);
    }

    // Face ACD: opposite vertex B
    let acd = ac.cross(ad);
    let acd_normal = if acd.dot(ab) > 0.0 { -acd } else { acd };
    if acd_normal.dot(ao) > 0.0 {
        simplex.set(&[d, c, a]);
        *direction = acd_normal;
        return handle_triangle_simplex(simplex, direction);
    }

    // Face BCD: opposite vertex A
    let bc = c - b;
    let bd = d - b;
    let bcd = bc.cross(bd);
    let bcd_normal = if bcd.dot(a - b) > 0.0 { -bcd } else { bcd };
    if bcd_normal.dot(-b) > 0.0 {
        simplex.set(&[d, c, b]);
        *direction = bcd_normal;
        return handle_triangle_simplex(simplex, direction);
    }

    // Origin is inside the tetrahedron
    true
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
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    // Minkowski difference support function
    let support = |dir: Vec3| -> Vec3 { support_a(dir) - support_b(-dir) };

    let mut simplex = Simplex::new();
    let mut direction = Vec3::new(1.0, 0.0, 0.0);

    // Get the first point
    let first_point = support(direction);
    if first_point.length_squared() < 1e-10 {
        // Origin is inside the Minkowski difference (or extremely close)
        simplex.push(first_point);
        return Some(simplex);
    }
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
    fn test_gjk_spheres_colliding() {
        let center_a = Vec3::new(0.0, 0.0, 0.0);
        let radius_a = 1.5;
        let center_b = Vec3::new(2.0, 0.0, 0.0);
        let radius_b = 1.5;

        let support_a = |dir: Vec3| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec3| center_b + dir.normalize_or_zero() * radius_b;

        let result = gjk(support_a, support_b, 30);
        assert!(result.is_some());
    }

    #[test]
    fn test_gjk_spheres_not_colliding() {
        let center_a = Vec3::new(0.0, 0.0, 0.0);
        let radius_a = 1.5;
        let center_b = Vec3::new(4.0, 0.0, 0.0);
        let radius_b = 1.5;

        let support_a = |dir: Vec3| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec3| center_b + dir.normalize_or_zero() * radius_b;

        let result = gjk(support_a, support_b, 30);
        assert!(result.is_none());
    }
}

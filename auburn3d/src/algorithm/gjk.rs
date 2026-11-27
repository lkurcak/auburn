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
///
/// Otherwise the simplex keeps only its part nearest the origin, and the search goes from
/// there straight towards the origin. Searching any other way can find points that bring the
/// simplex no closer, and then it cycles through the same states until it runs out of
/// iterations.
///
/// Only the parts that contain the newest point are candidates. The rest is the previous
/// simplex, which the newest point was found from, so the origin is nearer to something with
/// the newest point in it. Testing the rest anyway only lets rounding send the search back to
/// where it came from, which finds the newest point again, forever.
fn handle_simplex(simplex: &mut Simplex, direction: &mut Vec3) -> bool {
    let nearest = match simplex.count() {
        2 => nearest_on_segment(simplex.get(0), simplex.get(1), *direction),
        3 => nearest_on_triangle(simplex.get(0), simplex.get(1), simplex.get(2), *direction),
        4 => match nearest_on_tetrahedron(simplex, *direction) {
            Some(nearest) => nearest,
            None => return true,
        },
        _ => return false,
    };
    simplex.set(&nearest.points[..nearest.count]);
    *direction = nearest.direction;
    false
}

/// The part of a simplex nearest the origin, and the direction from it to the origin.
struct Nearest {
    /// Its corners, oldest first, the newest point last.
    points: [Vec3; 3],
    count: usize,
    /// About as long as the simplex's edges. A cubic triple product would underflow for tiny
    /// shapes, overflow for huge ones, and lose its precision in `Collider::support`, which
    /// adds a translation to it.
    direction: Vec3,
}

impl Nearest {
    fn point(a: Vec3) -> Self {
        Self {
            points: [a, Vec3::ZERO, Vec3::ZERO],
            count: 1,
            direction: -a,
        }
    }

    /// How far the origin is from this part of the simplex, whose newest point is `a`.
    fn distance(&self, a: Vec3) -> f32 {
        self.direction.normalize_or_zero().dot(-a)
    }
}

/// The part of the segment from `b` to the newest point `a` nearest the origin. `searched` is
/// the direction that found `a`.
fn nearest_on_segment(b: Vec3, a: Vec3, searched: Vec3) -> Nearest {
    let ab = b - a;
    let ao = -a;

    if ab.dot(ao) <= 0.0 {
        return Nearest::point(a);
    }
    // The origin is past a, towards b: search perpendicular to ab, towards the origin.
    let direction = match ab.cross(ao).try_normalize() {
        Some(normal) => normal.cross(ab),
        None => perpendicular_on_line(ab, searched),
    };
    Nearest {
        points: [b, a, Vec3::ZERO],
        count: 2,
        direction,
    }
}

/// A search direction perpendicular to `ab` for when the origin is on its line, where any
/// perpendicular will do.
///
/// It is also perpendicular to `searched`, the direction that found the newest point. If that
/// was already perpendicular to the line and found nothing new, as on a flat Minkowski
/// difference, the search turns a quarter around the line instead of repeating forever.
fn perpendicular_on_line(ab: Vec3, searched: Vec3) -> Vec3 {
    let turned = ab.normalize_or_zero().cross(searched.normalize_or_zero());
    if turned.length_squared() > 1e-6 {
        turned.normalize() * ab.length()
    } else {
        ab.any_orthogonal_vector()
    }
}

/// The part of the triangle `c`, `b`, `a` nearest the origin, `a` being the newest point.
fn nearest_on_triangle(c: Vec3, b: Vec3, a: Vec3, searched: Vec3) -> Nearest {
    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let Some(abc) = ab.cross(ac).try_normalize() else {
        // The points are on a line: keep the newest edge.
        return nearest_on_segment(b, a, searched);
    };

    // Outside an edge, in the triangle's plane, the origin is nearest that edge or a. Outside
    // both, it is nearest a, or the one edge that it is past a along.
    let outside_ab = ab.cross(abc).dot(ao) > 0.0;
    let outside_ac = abc.cross(ac).dot(ao) > 0.0;
    if outside_ab && (!outside_ac || ab.dot(ao) > 0.0) {
        return nearest_on_segment(b, a, searched);
    }
    if outside_ac {
        return nearest_on_segment(c, a, searched);
    }

    // Origin is in the triangle's prism; search towards the side it is on
    let normal = abc * ab.length();
    if normal.dot(ao) > 0.0 {
        Nearest {
            points: [c, b, a],
            count: 3,
            direction: normal,
        }
    } else {
        // Below the triangle; swap b and c to keep winding consistent
        Nearest {
            points: [b, c, a],
            count: 3,
            direction: -normal,
        }
    }
}

/// The part of the tetrahedron nearest the origin, or `None` if the origin is inside it.
///
/// The origin can be in front of up to three faces. Each one's nearest part is a candidate,
/// and the nearest of those wins.
fn nearest_on_tetrahedron(simplex: &Simplex, searched: Vec3) -> Option<Nearest> {
    let a = simplex.get(3);
    let b = simplex.get(2);
    let c = simplex.get(1);
    let d = simplex.get(0);
    let ao = -a;

    let mut nearest: Option<(Nearest, f32)> = None;
    // Each face with a, and the corner opposite it.
    for (p, q, opposite) in [(b, c, d), (c, d, b), (d, b, c)] {
        let Some(normal) = (p - a).cross(q - a).try_normalize() else {
            continue;
        };
        let outward = if normal.dot(opposite - a) > 0.0 {
            -normal
        } else {
            normal
        };
        if outward.dot(ao) <= 0.0 {
            continue;
        }
        let candidate = nearest_on_triangle(q, p, a, searched);
        let distance = candidate.distance(a);
        if nearest.as_ref().is_none_or(|(_, best)| distance < *best) {
            nearest = Some((candidate, distance));
        }
    }

    // Face BCD is the previous triangle, and a was found searching from it towards the
    // origin, so the origin is on a's side of it.
    nearest.map(|(nearest, _)| nearest)
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
    simplex.push(first_point);
    if first_point == Vec3::ZERO {
        // The origin is on the Minkowski difference: the shapes touch.
        return Some(simplex);
    }
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

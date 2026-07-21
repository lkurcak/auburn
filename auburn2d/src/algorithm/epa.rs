use super::gjk::Simplex;
/// Expanding Polytope Algorithm (EPA) for penetration depth calculation.
/// This algorithm extends GJK to find the penetration depth and direction when shapes overlap.
use crate::Vec2;

const EPA_MAX_ITERATIONS: usize = 30;
const EPA_TOLERANCE: f32 = 1e-4;

/// Represents an edge of the polytope with its distance and normal
#[derive(Debug, Clone, Copy)]
struct Edge {
    distance: f32,
    normal: Vec2,
    index: usize,
}

/// Run the EPA algorithm to find penetration depth and direction.
///
/// # Arguments
/// * `simplex` - Initial simplex from GJK (must contain the origin)
/// * `support_a` - Support function for shape A
/// * `support_b` - Support function for shape B
///
/// # Returns
/// * Penetration vector (direction and magnitude to separate the shapes)
pub fn epa<F1, F2>(initial_simplex: Simplex, support_a: F1, support_b: F2) -> Vec2
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    // Minkowski difference support function
    let support = |dir: Vec2| -> Vec2 { support_a(dir) - support_b(-dir) };

    // Convert simplex to polytope (list of vertices)
    let mut polytope: Vec<Vec2> = Vec::new();
    for i in 0..initial_simplex.count() {
        polytope.push(initial_simplex.get(i));
    }

    // Ensure the polytope is wound counter-clockwise
    if polytope.len() == 3 {
        let v0 = polytope[1] - polytope[0];
        let v1 = polytope[2] - polytope[0];
        let cross = v0.x * v1.y - v0.y * v1.x;
        if cross < 0.0 {
            polytope.swap(1, 2);
        }
    }

    for _ in 0..EPA_MAX_ITERATIONS {
        // Find the closest edge to the origin
        let edge = find_closest_edge(&polytope);

        // Get a new support point in the direction of the edge normal
        let support_point = support(edge.normal);

        // Check how much further we can go
        let distance = support_point.dot(edge.normal);

        // If we're not making progress, we've found the penetration depth
        if distance - edge.distance < EPA_TOLERANCE {
            // Return the penetration vector
            // Note: This points from A to B (direction to push B away from A)
            return edge.normal * distance;
        }

        // Insert the new point into the polytope
        polytope.insert(edge.index, support_point);
    }

    // If we didn't converge, return the best edge we found
    let edge = find_closest_edge(&polytope);
    edge.normal * edge.distance
}

/// Find the edge of the polytope closest to the origin
fn find_closest_edge(polytope: &[Vec2]) -> Edge {
    let mut closest_edge = Edge {
        distance: f32::MAX,
        normal: Vec2::ZERO,
        index: 0,
    };

    for i in 0..polytope.len() {
        let j = (i + 1) % polytope.len();

        let a = polytope[i];
        let b = polytope[j];

        let edge = b - a;

        // Normal perpendicular to the edge (pointing outward)
        let mut normal = Vec2::new(edge.y, -edge.x);
        let length = normal.length();
        if length > 1e-10 {
            normal /= length;
        } else {
            continue;
        }

        // Calculate distance from origin to this edge
        let distance = normal.dot(a);

        // Make sure normal points toward origin
        if distance < 0.0 {
            normal = -normal;
            let distance = -distance;

            if distance < closest_edge.distance {
                closest_edge = Edge {
                    distance,
                    normal,
                    index: j,
                };
            }
        } else if distance < closest_edge.distance {
            closest_edge = Edge {
                distance,
                normal,
                index: j,
            };
        }
    }

    closest_edge
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::algorithm::gjk::gjk;

    #[test]
    fn test_epa_circles_overlapping() {
        // Two circles with centers 1 unit apart, each with radius 1
        // Penetration should be ~1 unit
        let center_a = Vec2::new(0.0, 0.0);
        let radius_a = 1.0;
        let center_b = Vec2::new(1.0, 0.0);
        let radius_b = 1.0;

        let support_a = |dir: Vec2| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec2| center_b + dir.normalize_or_zero() * radius_b;

        let simplex = gjk(support_a, support_b, 30).expect("Should collide");
        let penetration = epa(simplex, support_a, support_b);

        // Check that penetration is approximately 1.0 in the x direction
        assert!((penetration.length() - 1.0).abs() < 0.1);
        assert!((penetration.normalize().dot(Vec2::X) - 1.0).abs() < 0.1);
    }
}

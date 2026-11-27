//! Expanding Polytope Algorithm (EPA) for penetration depth calculation.
//!
//! It extends GJK: starting from GJK's triangle around the origin, it grows a convex polygon
//! inside the Minkowski difference towards the origin's nearest boundary.
use super::gjk::Simplex;
use crate::Vec2;

const EPA_MAX_ITERATIONS: usize = 64;
/// EPA stops once the penetration it has found is no more than this fraction of the
/// polytope's size longer than the closest edge. Being relative, it works the same for tiny
/// and huge shapes.
const EPA_RELATIVE_TOLERANCE: f32 = 1e-5;
/// Far from the origin, rounding in the support points' world coordinates outweighs that, so
/// EPA also stops at this many units of f32 precision of those coordinates.
const EPA_ROUNDING_TOLERANCE: f32 = 4.0 * f32::EPSILON;

/// An edge of the polytope, from its vertex of the same index to the next one.
#[derive(Debug, Clone, Copy)]
struct Edge {
    /// The unit normal, pointing out of the polytope.
    normal: Vec2,
    /// How far the edge's line is from the origin, negative if the origin is outside it.
    /// Infinite for an edge too short to have a direction, so it is never the closest.
    distance: f32,
}

impl Edge {
    fn new(from: Vec2, to: Vec2) -> Self {
        let edge = to - from;
        // The polytope is counter-clockwise, so its outside is on the edge's right.
        let normal = Vec2::new(edge.y, -edge.x);
        let length = normal.length();
        if length > 0.0 {
            let normal = normal / length;
            Self {
                normal,
                distance: normal.dot(from),
            }
        } else {
            Self {
                normal: Vec2::ZERO,
                distance: f32::INFINITY,
            }
        }
    }
}

/// Run the EPA algorithm to find penetration depth and direction.
///
/// `initial_simplex` is the triangle GJK found around the origin. The result is a direction
/// times how far the Minkowski difference reaches along it, so moving the shapes apart by it
/// leaves them touching. When EPA doesn't converge it is the shortest such vector found so
/// far: slightly longer than the penetration depth, but not shorter.
pub fn epa<F1, F2>(initial_simplex: Simplex, support_a: F1, support_b: F2) -> Vec2
where
    F1: Fn(Vec2) -> Vec2,
    F2: Fn(Vec2) -> Vec2,
{
    // Minkowski difference support function, and the size of its world coordinates
    let support = |dir: Vec2| -> (Vec2, f32) {
        let (a, b) = (support_a(dir), support_b(-dir));
        (a - b, a.abs().max(b.abs()).max_element())
    };

    // GJK only returns fewer points when its first one is the origin: the shapes touch.
    if initial_simplex.count() < 3 {
        return Vec2::ZERO;
    }
    let mut vertices: Vec<Vec2> = (0..3).map(|i| initial_simplex.get(i)).collect();

    // Wind the triangle counter-clockwise, judging by its corners.
    let [p0, p1, p2] = [vertices[0], vertices[1], vertices[2]];
    let orientation = (p1 - p0).perp_dot(p2 - p0);
    if orientation == 0.0 || orientation.is_nan() {
        // Flat. GJK finds the last corner past the origin, off the others' line, unless the
        // origin is on the Minkowski difference's boundary: the shapes touch.
        return Vec2::ZERO;
    }
    if orientation < 0.0 {
        vertices.swap(1, 2);
    }
    let mut edges: Vec<Edge> = (0..3)
        .map(|i| Edge::new(vertices[i], vertices[(i + 1) % 3]))
        .collect();

    // Every vertex is on the Minkowski difference's boundary, so the farthest one bounds the
    // penetration depth from above.
    let size = vertices.iter().map(|v| v.length()).fold(0.0, f32::max);
    let relative_tolerance = EPA_RELATIVE_TOLERANCE * size;

    // The closest edge's distance is a lower bound on the penetration depth, and the support
    // point in its direction an upper one. EPA stops when they meet.
    let mut best = Vec2::ZERO;
    let mut best_depth = f32::INFINITY;

    for _ in 0..EPA_MAX_ITERATIONS {
        let Some(closest) = closest_edge(&edges) else {
            break;
        };
        let edge = edges[closest];

        let (point, world) = support(edge.normal);
        let depth = point.dot(edge.normal);
        if depth < best_depth {
            best_depth = depth;
            best = edge.normal * depth;
        }

        if best_depth - edge.distance <= relative_tolerance.max(EPA_ROUNDING_TOLERANCE * world) {
            break;
        }

        // Split the edge at the new point.
        let next = (closest + 1) % vertices.len();
        let (from, to) = (vertices[closest], vertices[next]);
        vertices.insert(closest + 1, point);
        edges[closest] = Edge::new(from, point);
        edges.insert(closest + 1, Edge::new(point, to));
    }

    best
}

/// The index of the edge closest to the origin, if any edge has a direction.
fn closest_edge(edges: &[Edge]) -> Option<usize> {
    let (index, edge) = edges
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| a.distance.total_cmp(&b.distance))?;
    edge.distance.is_finite().then_some(index)
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

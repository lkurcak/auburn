//! Expanding Polytope Algorithm (EPA) for penetration depth calculation.
use super::gjk::Simplex;
use crate::Vec3;

const EPA_MAX_ITERATIONS: usize = 64;
/// EPA stops once the penetration it has found is no more than this fraction of the
/// polytope's size longer than the closest face. Being relative, it works the same for tiny
/// and huge shapes.
const EPA_RELATIVE_TOLERANCE: f32 = 1e-5;
/// Far from the origin, rounding in the support points' world coordinates outweighs that, so
/// EPA also stops at this many units of f32 precision of those coordinates.
const EPA_ROUNDING_TOLERANCE: f32 = 4.0 * f32::EPSILON;

/// A triangle of the polytope.
#[derive(Debug, Clone, Copy)]
struct Face {
    /// Indices of the corners, counter-clockwise seen from outside the polytope.
    vertices: [usize; 3],
    /// `neighbors[k]` is the face across [`Face::edge`] `k`.
    neighbors: [usize; 3],
    /// The unit normal, pointing out of the polytope. Zero if the triangle is too thin to
    /// have a reliable one.
    normal: Vec3,
    /// How far the face's plane is from the origin, negative if the origin is outside it.
    /// Infinite for a degenerate face, so it is never the closest.
    distance: f32,
    removed: bool,
}

impl Face {
    fn new(points: &[Vec3], vertices: [usize; 3], neighbors: [usize; 3]) -> Self {
        let [a, b, c] = vertices.map(|i| points[i]);
        let (ab, ac) = (b - a, c - a);
        let cross = ab.cross(ac);
        let length_squared = cross.length_squared();
        // A cross product no bigger than its rounding error has no direction. NaN fails the
        // comparison too.
        let noise = f32::EPSILON * f32::EPSILON * ab.length_squared() * ac.length_squared();
        let (normal, distance) = if length_squared > noise {
            let normal = cross / length_squared.sqrt();
            (normal, normal.dot(a))
        } else {
            (Vec3::ZERO, f32::INFINITY)
        };
        Self {
            vertices,
            neighbors,
            normal,
            distance,
            removed: false,
        }
    }

    /// Edge `k` runs from corner `k` to the next one.
    fn edge(&self, k: usize) -> (usize, usize) {
        (self.vertices[k], self.vertices[(k + 1) % 3])
    }

    /// Whether `point` is in front of the face, or within `coplanar` of its plane: faces that
    /// have the new point on their plane go too, so that no new face is a sliver. A
    /// degenerate face always goes when its neighbour does.
    fn sees(&self, points: &[Vec3], point: Vec3, coplanar: f32) -> bool {
        self.normal == Vec3::ZERO || self.normal.dot(point - points[self.vertices[0]]) > -coplanar
    }
}

/// An edge on the horizon: the boundary between the faces a new point sees and those it
/// doesn't. `outside` is the face that stays.
#[derive(Debug, Clone, Copy)]
struct HorizonEdge {
    from: usize,
    to: usize,
    outside: usize,
}

/// Run the EPA algorithm to find penetration depth and direction.
///
/// `initial_simplex` is the tetrahedron GJK found around the origin. The result is a
/// direction times how far the Minkowski difference reaches along it, so moving the shapes
/// apart by it leaves them touching. When EPA doesn't converge it is the shortest such vector
/// found so far: slightly longer than the penetration depth, but not shorter.
pub fn epa<F1, F2>(initial_simplex: Simplex, support_a: F1, support_b: F2) -> Vec3
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    // Minkowski difference support function, and the size of its world coordinates
    let support = |dir: Vec3| -> (Vec3, f32) {
        let (a, b) = (support_a(dir), support_b(-dir));
        (a - b, a.abs().max(b.abs()).max_element())
    };

    // GJK only returns fewer points when its first one is the origin: the shapes touch.
    if initial_simplex.count() < 4 {
        return Vec3::ZERO;
    }
    let mut points: Vec<Vec3> = (0..4).map(|i| initial_simplex.get(i)).collect();

    // Orient the tetrahedron by its corners, not by the origin, which can be on a face's
    // plane: the last corner goes behind the first face.
    let [p0, p1, p2, p3] = [points[0], points[1], points[2], points[3]];
    let orientation = (p1 - p0).cross(p2 - p0).dot(p3 - p0);
    if orientation == 0.0 || orientation.is_nan() {
        // Flat. GJK finds the last corner past the origin, off the others' plane, unless the
        // origin is on the Minkowski difference's boundary: the shapes touch.
        return Vec3::ZERO;
    }
    if orientation > 0.0 {
        points.swap(1, 2);
    }
    let mut faces = vec![
        Face::new(&points, [0, 1, 2], [1, 2, 3]),
        Face::new(&points, [0, 3, 1], [3, 2, 0]),
        Face::new(&points, [1, 3, 2], [1, 3, 0]),
        Face::new(&points, [2, 3, 0], [2, 1, 0]),
    ];

    // Every vertex is on the Minkowski difference's boundary, so the farthest one bounds the
    // penetration depth from above.
    let size = points.iter().map(|v| v.length()).fold(0.0, f32::max);
    let relative_tolerance = EPA_RELATIVE_TOLERANCE * size;

    // The closest face's distance is a lower bound on the penetration depth, and the support
    // point in its direction an upper one. EPA stops when they meet.
    let mut best = Vec3::ZERO;
    let mut best_depth = f32::INFINITY;
    let mut horizon = Vec::new();

    for _ in 0..EPA_MAX_ITERATIONS {
        let Some(closest) = closest_face(&faces) else {
            break;
        };
        let face = faces[closest];

        let (point, world) = support(face.normal);
        let depth = point.dot(face.normal);
        if depth < best_depth {
            best_depth = depth;
            best = face.normal * depth;
        }

        let rounding = EPA_ROUNDING_TOLERANCE * world;
        if best_depth - face.distance <= relative_tolerance.max(rounding) {
            break;
        }

        // Remove the faces the new point sees, then close the hole with faces from its rim
        // to the point.
        let index = points.len();
        points.push(point);
        horizon.clear();
        faces[closest].removed = true;
        for k in 0..3 {
            carve(&mut faces, &points, closest, k, rounding, &mut horizon);
        }
        if !is_loop(&horizon) {
            // Rounding made the visible faces something other than a disk. Patching that
            // would break the polytope, so settle for the best answer so far.
            break;
        }

        let first = faces.len();
        let n = horizon.len();
        for (i, edge) in horizon.iter().enumerate() {
            let next = first + (i + 1) % n;
            let previous = first + (i + n - 1) % n;
            faces.push(Face::new(
                &points,
                [edge.from, edge.to, index],
                [edge.outside, next, previous],
            ));
            let outside = &mut faces[edge.outside];
            let k = find_edge(outside, edge.to, edge.from);
            outside.neighbors[k] = first + i;
        }
        debug_assert!(is_closed(&faces), "EPA's polytope has a hole");
    }

    best
}

/// Whether every kept face's neighbours are kept too and share its edges, the other way
/// round. Checked in debug builds.
fn is_closed(faces: &[Face]) -> bool {
    let kept = faces.iter().enumerate().filter(|(_, face)| !face.removed);
    kept.into_iter().all(|(i, face)| {
        (0..3).all(|k| {
            let (from, to) = face.edge(k);
            let neighbor = &faces[face.neighbors[k]];
            !neighbor.removed
                && (0..3).any(|j| neighbor.edge(j) == (to, from) && neighbor.neighbors[j] == i)
        })
    })
}

/// Across edge `k` of the removed face `from`, removes the faces that see the newest point,
/// and records the edges where it stops in `horizon`, in order around the hole.
///
/// Each removed face goes on with the edges after the one it was entered by, so the horizon
/// comes out counter-clockwise, each edge starting where the previous one ends.
fn carve(
    faces: &mut [Face],
    points: &[Vec3],
    from: usize,
    k: usize,
    coplanar: f32,
    horizon: &mut Vec<HorizonEdge>,
) {
    let (a, b) = faces[from].edge(k);
    let next = faces[from].neighbors[k];
    if faces[next].removed {
        return;
    }
    let point = points[points.len() - 1];
    if faces[next].sees(points, point, coplanar) {
        faces[next].removed = true;
        let entered = find_edge(&faces[next], b, a);
        carve(faces, points, next, (entered + 1) % 3, coplanar, horizon);
        carve(faces, points, next, (entered + 2) % 3, coplanar, horizon);
    } else {
        horizon.push(HorizonEdge {
            from: a,
            to: b,
            outside: next,
        });
    }
}

/// Whether the horizon is one closed loop, so that new faces from it to the new point close
/// the polytope.
fn is_loop(horizon: &[HorizonEdge]) -> bool {
    horizon.len() >= 3
        && horizon
            .iter()
            .zip(horizon.iter().cycle().skip(1))
            .all(|(edge, next)| edge.to == next.from)
}

/// The index of `face`'s edge from vertex `from` to vertex `to`. Neighbouring faces always
/// share their edge in opposite directions.
fn find_edge(face: &Face, from: usize, to: usize) -> usize {
    (0..3)
        .find(|&k| face.edge(k) == (from, to))
        .expect("neighbouring faces share an edge")
}

/// The face whose plane is closest to the origin, if any face is not degenerate.
fn closest_face(faces: &[Face]) -> Option<usize> {
    faces
        .iter()
        .enumerate()
        .filter(|(_, face)| !face.removed && face.distance.is_finite())
        .min_by(|(_, a), (_, b)| a.distance.total_cmp(&b.distance))
        .map(|(i, _)| i)
}

#[cfg(test)]
mod tests {
    use super::super::gjk::gjk;
    use super::*;

    #[test]
    fn test_epa_spheres_overlapping() {
        let center_a = Vec3::new(0.0, 0.0, 0.0);
        let radius_a = 1.0;
        let center_b = Vec3::new(1.0, 0.0, 0.0);
        let radius_b = 1.0;

        let support_a = |dir: Vec3| center_a + dir.normalize_or_zero() * radius_a;
        let support_b = |dir: Vec3| center_b + dir.normalize_or_zero() * radius_b;

        let simplex = gjk(support_a, support_b, 30).expect("Should collide");
        let penetration = epa(simplex, support_a, support_b);

        // Check that penetration is approximately 1.0 in the x direction
        assert!(
            (penetration.length() - 1.0).abs() < 0.1,
            "penetration length: {:?}",
            penetration.length()
        );
        assert!(
            (penetration.normalize().dot(Vec3::X) - 1.0).abs() < 0.1,
            "penetration direction: {:?}",
            penetration
        );
    }
}

use super::gjk::Simplex;
/// Expanding Polytope Algorithm (EPA) for penetration depth calculation.
use crate::Vec3;

const EPA_MAX_ITERATIONS: usize = 30;
const EPA_TOLERANCE: f32 = 1e-4;

/// A face of the polytope, defined by three vertex indices.
#[derive(Debug, Clone, Copy)]
struct Face {
    indices: [usize; 3],
    normal: Vec3,
    distance: f32,
}

/// Run the EPA algorithm to find penetration depth and direction.
pub fn epa<F1, F2>(initial_simplex: Simplex, support_a: F1, support_b: F2) -> Vec3
where
    F1: Fn(Vec3) -> Vec3,
    F2: Fn(Vec3) -> Vec3,
{
    // Minkowski difference support function
    let support = |dir: Vec3| -> Vec3 { support_a(dir) - support_b(-dir) };

    // Convert simplex to polytope
    let mut vertices: Vec<Vec3> = Vec::new();
    for i in 0..initial_simplex.count() {
        vertices.push(initial_simplex.get(i));
    }

    let mut faces = if vertices.len() == 4 {
        let mut faces = Vec::new();
        let face_indices = [
            [0, 1, 2],
            [0, 3, 1],
            [0, 2, 3],
            [1, 3, 2],
        ];
        for &idx in &face_indices {
            let a = vertices[idx[0]];
            let b = vertices[idx[1]];
            let c = vertices[idx[2]];
            let mut normal = compute_face_normal(a, b, c);
            // Ensure normal points outward: origin should be on the negative side
            if normal.dot(a) < 0.0 {
                normal = -normal;
            }
            faces.push(Face {
                indices: idx,
                normal,
                distance: normal.dot(a),
            });
        }
        faces
    } else {
        Vec::new()
    };

    for _ in 0..EPA_MAX_ITERATIONS {
        if faces.is_empty() {
            break;
        }

        // Find the closest face to the origin
        let closest_idx = find_closest_face(&faces);
        let closest = faces[closest_idx];

        // Get a new support point in the direction of the closest face normal
        let support_point = support(closest.normal);

        // Check how much further we can go
        let distance = support_point.dot(closest.normal);

        if distance - closest.distance < EPA_TOLERANCE {
            return closest.normal * distance;
        }

        // Insert the new point
        let new_idx = vertices.len();
        vertices.push(support_point);

        // Remove faces that can see the new point and keep their silhouette edges
        let mut edges: Vec<(usize, usize)> = Vec::new();
        let mut new_faces = Vec::new();
        for face in faces {
            let a = vertices[face.indices[0]];
            let b = vertices[face.indices[1]];
            let c = vertices[face.indices[2]];
            let center = (a + b + c) / 3.0;
            let to_point = support_point - center;
            if to_point.dot(face.normal) > 0.0 {
                // Face can see the new point; remove it and add its edges
                add_edge(&mut edges, face.indices[0], face.indices[1]);
                add_edge(&mut edges, face.indices[1], face.indices[2]);
                add_edge(&mut edges, face.indices[2], face.indices[0]);
            } else {
                new_faces.push(face);
            }
        }

        // Create new faces from the silhouette edges to the new point
        for (i, j) in edges {
            let a = vertices[i];
            let b = vertices[j];
            let mut normal = compute_face_normal(a, b, support_point);
            // Ensure normal points outward
            if normal.dot(a) < 0.0 {
                normal = -normal;
            }
            new_faces.push(Face {
                indices: [i, j, new_idx],
                normal,
                distance: normal.dot(a),
            });
        }

        faces = new_faces;
    }

    // If we didn't converge, return the best face we found
    if !faces.is_empty() {
        let closest = faces[find_closest_face(&faces)];
        closest.normal * closest.distance
    } else {
        Vec3::ZERO
    }
}

fn compute_face_normal(a: Vec3, b: Vec3, c: Vec3) -> Vec3 {
    let ab = b - a;
    let ac = c - a;
    let mut normal = ac.cross(ab);
    let length = normal.length();
    if length > 1e-10 {
        normal /= length;
    }
    normal
}

fn find_closest_face(faces: &[Face]) -> usize {
    let mut closest = 0;
    let mut min_distance = f32::MAX;
    for (i, face) in faces.iter().enumerate() {
        if face.distance < min_distance {
            min_distance = face.distance;
            closest = i;
        }
    }
    closest
}

fn add_edge(edges: &mut Vec<(usize, usize)>, a: usize, b: usize) {
    // Remove the edge if it already exists (with opposite direction)
    let existing = edges.iter().position(|(x, y)| *x == b && *y == a);
    if let Some(idx) = existing {
        edges.remove(idx);
    } else {
        edges.push((a, b));
    }
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

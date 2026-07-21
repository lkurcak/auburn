use crate::Vec2;

/// Check if a point is inside or on the boundary of a convex polygon.
pub fn point_in_convex_polygon(point: Vec2, vertices: &[Vec2]) -> bool {
    let n = vertices.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        let j = (i + 1) % n;
        let edge = vertices[j] - vertices[i];
        let to_point = point - vertices[i];
        // For a CCW polygon, the interior is to the left of each edge.
        if edge.perp_dot(to_point) < -1e-6 {
            return false;
        }
    }
    true
}

/// Collect all times t where the ray `origin + vel * t` intersects an edge of the convex polygon.
fn ray_polygon_times(origin: Vec2, vel: Vec2, vertices: &[Vec2]) -> Vec<f32> {
    let mut times = Vec::new();
    let n = vertices.len();
    for i in 0..n {
        let p1 = vertices[i];
        let p2 = vertices[(i + 1) % n];
        let edge = p2 - p1;
        let r = origin - p1;
        let denom = vel.perp_dot(edge);
        if denom.abs() < 1e-10 {
            continue;
        }
        let t = -r.perp_dot(edge) / denom;
        let s = vel.perp_dot(r) / denom;
        if s >= -1e-6 && s <= 1.0 + 1e-6 {
            times.push(t);
        }
    }
    times
}

/// Unbounded TOI for a moving point vs a convex polygon.
/// Returns the earliest intersection time (can be negative).
pub fn point_poly_tttoi(origin: Vec2, vel: Vec2, vertices: &[Vec2]) -> Option<f32> {
    if vel.length_squared() < 1e-10 {
        return if point_in_convex_polygon(origin, vertices) {
            Some(0.0)
        } else {
            None
        };
    }
    let times = ray_polygon_times(origin, vel, vertices);
    if times.is_empty() {
        return None;
    }
    Some(times.iter().copied().fold(f32::INFINITY, f32::min))
}

/// Unbounded TOI and exit for a moving point vs a convex polygon.
pub fn point_poly_tttoiae(origin: Vec2, vel: Vec2, vertices: &[Vec2]) -> Option<(f32, f32)> {
    if vel.length_squared() < 1e-10 {
        return if point_in_convex_polygon(origin, vertices) {
            Some((0.0, 0.0))
        } else {
            None
        };
    }
    let times = ray_polygon_times(origin, vel, vertices);
    if times.is_empty() {
        return None;
    }
    let min_t = times.iter().copied().fold(f32::INFINITY, f32::min);
    let max_t = times.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    Some((min_t, max_t))
}

/// Bounded TOI for a moving point vs a convex polygon.
pub fn point_poly_toi(origin: Vec2, vel: Vec2, vertices: &[Vec2]) -> Option<f32> {
    if vel.length_squared() < 1e-10 {
        return if point_in_convex_polygon(origin, vertices) {
            Some(0.0)
        } else {
            None
        };
    }
    let times = ray_polygon_times(origin, vel, vertices);
    if times.is_empty() {
        return None;
    }
    let min_t = times.iter().copied().fold(f32::INFINITY, f32::min);
    let max_t = times.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    if min_t <= 0.0 && max_t >= 0.0 {
        Some(0.0)
    } else if min_t > 0.0 {
        Some(min_t)
    } else {
        None
    }
}

/// Bounded TOI and exit for a moving point vs a convex polygon.
pub fn point_poly_toiae(origin: Vec2, vel: Vec2, vertices: &[Vec2]) -> Option<(f32, f32)> {
    if vel.length_squared() < 1e-10 {
        return if point_in_convex_polygon(origin, vertices) {
            Some((0.0, 0.0))
        } else {
            None
        };
    }
    let times = ray_polygon_times(origin, vel, vertices);
    if times.is_empty() {
        return None;
    }
    let min_t = times.iter().copied().fold(f32::INFINITY, f32::min);
    let max_t = times.iter().copied().fold(f32::NEG_INFINITY, f32::max);
    if min_t <= 0.0 && max_t >= 0.0 {
        Some((0.0, max_t))
    } else if min_t > 0.0 {
        Some((min_t, max_t))
    } else {
        None
    }
}

// --- Swept SAT helpers ---

/// Given two 1D intervals and relative velocity along that axis, returns the time interval
/// during which the intervals overlap. If they can never overlap, returns None.
fn axis_overlap_interval(
    min_a: f32,
    max_a: f32,
    min_b: f32,
    max_b: f32,
    v: f32,
) -> Option<(f32, f32)> {
    if v.abs() < 1e-10 {
        if max_a < min_b || min_a > max_b {
            return None;
        }
        Some((f32::NEG_INFINITY, f32::INFINITY))
    } else {
        let t1 = (max_b - min_a) / v;
        let t2 = (min_b - max_a) / v;
        Some((t1.min(t2), t1.max(t2)))
    }
}

/// Swept SAT for two shapes given as projection functions and a list of separating axes.
/// Returns the unbounded overlap interval (t_enter, t_exit).
pub fn swept_sat_ttoiae<F1, F2>(
    get_proj_a: F1,
    get_proj_b: F2,
    axes: &[Vec2],
    vel: Vec2,
) -> Option<(f32, f32)>
where
    F1: Fn(Vec2) -> (f32, f32),
    F2: Fn(Vec2) -> (f32, f32),
{
    let mut t_enter = f32::NEG_INFINITY;
    let mut t_exit = f32::INFINITY;

    for &axis in axes {
        let (min_a, max_a) = get_proj_a(axis);
        let (min_b, max_b) = get_proj_b(axis);
        let v = vel.dot(axis);

        let (t0, t1) = axis_overlap_interval(min_a, max_a, min_b, max_b, v)?;
        t_enter = t_enter.max(t0);
        t_exit = t_exit.min(t1);

        if t_enter > t_exit {
            return None;
        }
    }

    Some((t_enter, t_exit))
}

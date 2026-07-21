/// Example: Adding a new Triangle shape to Auburn2D
///
/// This demonstrates how simple it is to add new shapes with the GJK/EPA refactoring.
/// You only need to implement the Support trait!
use auburn2d::prelude::*;

/// A triangle shape defined by three vertices (relative to center)
#[derive(Debug, Clone, Copy)]
pub struct Triangle {
    pub vertices: [Vec2; 3],
}

impl Triangle {
    pub fn new(v0: Vec2, v1: Vec2, v2: Vec2) -> Self {
        Self {
            vertices: [v0, v1, v2],
        }
    }

    /// Create an equilateral triangle with the given radius
    pub fn equilateral(radius: f32) -> Self {
        use std::f32::consts::PI;
        let angle_step = 2.0 * PI / 3.0;
        Self::new(
            Vec2::new(
                radius * (0.0 * angle_step).cos(),
                radius * (0.0 * angle_step).sin(),
            ),
            Vec2::new(
                radius * (1.0 * angle_step).cos(),
                radius * (1.0 * angle_step).sin(),
            ),
            Vec2::new(
                radius * (2.0 * angle_step).cos(),
                radius * (2.0 * angle_step).sin(),
            ),
        )
    }
}

// Implement the Shape trait (required for all shapes)
impl auburn2d::shape::Shape for Triangle {
    fn area(&self) -> f32 {
        // Calculate triangle area using cross product
        let v0 = self.vertices[0];
        let v1 = self.vertices[1];
        let v2 = self.vertices[2];
        ((v1 - v0).perp_dot(v2 - v0)).abs() * 0.5
    }
}

// Implement the Support trait - this is the only shape-specific code needed!
impl Support for Triangle {
    fn support(&self, dir: Vec2) -> Vec2 {
        // Find the vertex that is furthest in the given direction
        let mut max_vertex = self.vertices[0];
        let mut max_dot = self.vertices[0].dot(dir);

        for &vertex in &self.vertices[1..] {
            let dot = vertex.dot(dir);
            if dot > max_dot {
                max_dot = dot;
                max_vertex = vertex;
            }
        }

        max_vertex
    }
}

// That's it! Now Triangle automatically works with collision detection!

fn main() {
    println!("=== Auburn2D Triangle Example ===\n");

    // Create a triangle
    let triangle = Triangle::equilateral(1.0);
    let triangle_pos = Vec2::new(0.0, 0.0);
    let triangle_collider = triangle.at(&triangle_pos);

    // Create a ball
    let ball = Ball::new(0.5);
    let ball_pos = Vec2::new(1.0, 0.0);
    let ball_collider = ball.at(&ball_pos);

    // Test collision - automatically works via GJK!
    println!("Triangle at {:?} vs Ball at {:?}", triangle_pos, ball_pos);
    if triangle_collider.collides(&ball_collider) {
        println!("✅ Collision detected!");

        // Get penetration depth - automatically works via EPA!
        if let Some(penetration) = triangle_collider.penetrates(&ball_collider) {
            println!("   Penetration vector: {:?}", penetration);
            println!("   Penetration depth: {:.3}", penetration.length());
        }
    } else {
        println!("❌ No collision");
    }

    println!();

    // Test with a box
    let box2d = Box2d::square(0.8);
    let box_pos = Vec2::new(0.5, 0.5);
    let box_collider = box2d.at(&box_pos);

    println!("Triangle at {:?} vs Box at {:?}", triangle_pos, box_pos);
    if triangle_collider.collides(&box_collider) {
        println!("✅ Collision detected!");

        if let Some(penetration) = triangle_collider.penetrates(&box_collider) {
            println!("   Penetration vector: {:?}", penetration);
            println!("   Penetration depth: {:.3}", penetration.length());
        }
    } else {
        println!("❌ No collision");
    }

    println!();

    // Test triangle vs triangle
    let triangle2 = Triangle::equilateral(0.8);
    let triangle2_pos = Vec2::new(0.3, 0.3);
    let triangle2_collider = triangle2.at(&triangle2_pos);

    println!(
        "Triangle at {:?} vs Triangle at {:?}",
        triangle_pos, triangle2_pos
    );
    if triangle_collider.collides(&triangle2_collider) {
        println!("✅ Collision detected!");

        if let Some(penetration) = triangle_collider.penetrates(&triangle2_collider) {
            println!("   Penetration vector: {:?}", penetration);
            println!("   Penetration depth: {:.3}", penetration.length());
        }
    } else {
        println!("❌ No collision");
    }

    println!("\n🎉 Triangle shape works with all existing shapes automatically!");
    println!("   No N×N implementations needed - just implement Support!");
}

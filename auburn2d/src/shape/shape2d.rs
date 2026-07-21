use crate::Vec2;
use crate::prelude::*;
use crate::transformation::Transform2d;

#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "bevy", derive(bevy::prelude::Component))]
pub enum Shape2d {
    Point(Point),
    Ball(Ball),
    Box2d(Box2d),
}

impl Shape2d {
    pub fn point() -> Self {
        Self::Point(Point)
    }

    pub fn ball(radius: f32) -> Self {
        Self::Ball(Ball::new(radius))
    }

    pub fn square(half_width: f32) -> Self {
        Self::Box2d(Box2d::new(Vec2::new(half_width, half_width)))
    }

    pub fn box2d(half_width: f32, half_height: f32) -> Self {
        Self::Box2d(Box2d::new(Vec2::new(half_width, half_height)))
    }

    pub fn at<'a, I: crate::transformation::Transformation2d>(
        &'a self,
        isometry: &'a I,
    ) -> crate::collider::Collider<'a, Self, I> {
        crate::shape::Shape::at(self, isometry)
    }
}

impl Shape for Shape2d {
    fn area(&self) -> f32 {
        match self {
            Shape2d::Ball(ball) => ball.area(),
            Shape2d::Box2d(box2d) => box2d.area(),
            Shape2d::Point(point) => point.area(),
        }
    }
}

impl Support for Shape2d {
    fn support(&self, dir: Vec2) -> Vec2 {
        match self {
            Shape2d::Ball(ball) => ball.support(dir),
            Shape2d::Box2d(box2d) => box2d.support(dir),
            Shape2d::Point(point) => point.support(dir),
        }
    }
}

// NOTE: Collides and Penetrates now use the generic GJK/EPA implementations
// But TimeOfImpact still needs specific implementations because we don't have
// a generic conservative advancement implementation integrated into the trait system yet.

macro_rules! shape2d_against_the_world {
    ($trait:ident; $fn:ident; $tr:ty; $out:ty; ($($args:tt)*); ($($args_out:tt)*)) => {
        impl<'a, S: Shape> $trait<Collider<'a, S, $tr>> for Collider<'a, Shape2d, $tr>
        where
            Collider<'a, Point, $tr>: $trait<Collider<'a, S, $tr>>,
            Collider<'a, Ball, $tr>: $trait<Collider<'a, S, $tr>>,
            Collider<'a, Box2d, $tr>: $trait<Collider<'a, S, $tr>>,
        {
            fn $fn(&self, other: &Collider<'a, S, $tr>, $($args)*) -> $out {
                match self.shape {
                    Shape2d::Point(point) => Collider::new(point, self.isometry).$fn(other, $($args_out)*),
                    Shape2d::Ball(ball) => Collider::new(ball, self.isometry).$fn(other, $($args_out)*),
                    Shape2d::Box2d(box2d) => Collider::new(box2d, self.isometry).$fn(other, $($args_out)*),
                }
            }
        }
    };
}

shape2d_against_the_world!(TimeOfImpact; toi; Vec2; Option<f32>; (vel: Vec2); (vel));
shape2d_against_the_world!(TimeOfImpact; toi; Transform2d; Option<f32>; (vel: Vec2); (vel));
shape2d_against_the_world!(TimeOfImpactAndExit; toiae; Vec2; Option<(f32, f32)>; (vel: Vec2); (vel));
shape2d_against_the_world!(TimeOfImpactAndExit; toiae; Transform2d; Option<(f32, f32)>; (vel: Vec2); (vel));

macro_rules! thing_against_shape2d {
    ($trait:ident; $fn:ident; $tr:ty; $out:ty; ($($args:tt)*); ($($args_out:tt)*); $shape:ty) => {
        impl<'a> $trait<Collider<'a, Shape2d, $tr>> for Collider<'a, $shape, $tr>
        where
            Collider<'a, $shape, $tr>: $trait<Collider<'a, Point, $tr>>,
            Collider<'a, $shape, $tr>: $trait<Collider<'a, Ball, $tr>>,
            Collider<'a, $shape, $tr>: $trait<Collider<'a, Box2d, $tr>>,
        {
            fn $fn(&self, other: &Collider<'a, Shape2d, $tr>, $($args)*) -> $out {
                match other.shape {
                    Shape2d::Point(point) => self.$fn(&Collider::new(point, other.isometry), $($args_out)*),
                    Shape2d::Ball(ball) => self.$fn(&Collider::new(ball, other.isometry), $($args_out)*),
                    Shape2d::Box2d(box2d) => self.$fn(&Collider::new(box2d, other.isometry), $($args_out)*),
                }
            }
        }
    };
}

macro_rules! things_against_shape2d {
    ($($shape:ty),+) => {
        $(
            thing_against_shape2d!(TimeOfImpact; toi; Vec2; Option<f32>; (vel: Vec2); (vel); $shape);
            thing_against_shape2d!(TimeOfImpact; toi; Transform2d; Option<f32>; (vel: Vec2); (vel); $shape);
            thing_against_shape2d!(TimeOfImpactAndExit; toiae; Vec2; Option<(f32, f32)>; (vel: Vec2); (vel); $shape);
            thing_against_shape2d!(TimeOfImpactAndExit; toiae; Transform2d; Option<(f32, f32)>; (vel: Vec2); (vel); $shape);
        )+
    };
}

things_against_shape2d!(Point, Ball, Box2d);

use super::Transformation2d;
use crate::Vec2;

impl Transformation2d for Vec2 {
    fn apply_to_origin(&self) -> Vec2 {
        *self
    }

    fn apply(&self, point: Vec2) -> Vec2 {
        self + point
    }

    fn inverse(&self) -> Self {
        -self
    }

    fn compose(&self, other: &Self) -> Self {
        // Standard convention: a.compose(&b) means apply b first, then a.
        // So (a ∘ b)(x) = a(b(x)) = a + (b + x) = b + a + x
        other + *self
    }
}

#[cfg(test)]
mod tests {
    use super::Transformation2d;
    use crate::Vec2;

    #[test]
    fn apply_to_origin() {
        let isometry = Vec2::new(1.0, 2.0);
        assert_eq!(isometry.apply_to_origin(), Vec2::new(1.0, 2.0));
    }

    #[test]
    fn apply() {
        let isometry = Vec2::new(1.0, 2.0);
        assert_eq!(isometry.apply(Vec2::ONE), Vec2::new(2.0, 3.0));
    }

    #[test]
    fn inverse() {
        let isometry = Vec2::new(1.0, 2.0);
        assert_eq!(isometry.inverse(), Vec2::new(-1.0, -2.0));
    }

    #[test]
    fn compose() {
        let a = Vec2::new(1.0, 2.0);
        let b = Vec2::new(3.0, 4.0);
        assert_eq!(a.compose(&b), Vec2::new(4.0, 6.0));

        let point = Vec2::new(5.0, 7.0);
        assert_eq!(a.apply(b.apply(point)), a.compose(&b).apply(point));
    }
}

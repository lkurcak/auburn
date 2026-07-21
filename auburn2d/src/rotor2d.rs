use crate::Vec2;
use std::ops::Mul;

#[derive(Debug, Clone, Copy, PartialEq)]
/// Represents a 2D rotation.
///
/// `Rotor2d` is used to represent rotations in 2D space. It is more stable and efficient
/// than using a single angle, avoiding issues like gimbal lock (though less relevant in 2D)
/// and providing easy composition.
///
/// # Examples
///
/// ```
/// use auburn2d::prelude::*;
/// use auburn2d::rotor2d::Rotor2d;
/// use auburn2d::Vec2;
/// use std::f32::consts::PI;
///
/// let rotor = Rotor2d::radians(PI / 2.0);
/// let v = Vec2::new(1.0, 0.0);
/// let rotated = rotor * v;
///
/// assert!((rotated.x - 0.0).abs() < 1e-6);
/// assert!((rotated.y - 1.0).abs() < 1e-6);
/// ```
pub struct Rotor2d {
    a: Vec2,
}

impl Default for Rotor2d {
    fn default() -> Self {
        Self {
            a: Vec2::new(1.0, 0.0),
        }
    }
}

impl Rotor2d {
    /// Identity rotor (no rotation).
    pub const IDENTITY: Self = Self {
        a: Vec2::new(1.0, 0.0),
    };

    /// Create a new rotor from an angle in radians.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::rotor2d::Rotor2d;
    /// use std::f32::consts::PI;
    ///
    /// let rotor = Rotor2d::radians(PI / 2.0);
    /// assert!((rotor.angle() - PI / 2.0).abs() < 1e-6);
    /// ```
    pub fn radians(angle: f32) -> Self {
        let (sin, cos) = angle.sin_cos();
        Self {
            a: Vec2::new(cos, sin),
        }
    }

    /// Get the angle of the rotor in radians.
    ///
    /// The returned angle is in the range [-PI, PI].
    pub fn angle(&self) -> f32 {
        self.a.y.atan2(self.a.x)
    }

    /// Get the inverse of the rotor.
    ///
    /// The inverse rotor rotates in the opposite direction.
    ///
    /// # Examples
    ///
    /// ```
    /// use auburn2d::prelude::*;
    /// use auburn2d::rotor2d::Rotor2d;
    /// use auburn2d::Vec2;
    /// use std::f32::consts::PI;
    ///
    /// let rotor = Rotor2d::radians(PI / 2.0);
    /// let inverse = rotor.inverse();
    /// let v = Vec2::new(0.0, 1.0);
    /// let rotated = inverse * v;
    ///
    /// assert!((rotated.x - 1.0).abs() < 1e-6);
    /// assert!((rotated.y - 0.0).abs() < 1e-6);
    /// ```
    pub fn inverse(&self) -> Self {
        Self {
            a: Vec2::new(self.a.x, -self.a.y),
        }
    }

    /// Create a rotor from a quaternion.
    ///
    /// This ignores any rotation component that is not around the Z axis.
    pub fn from_quaternion(quat: crate::Quat) -> Rotor2d {
        let norm = quat.dot(quat).sqrt();
        let w = quat.w / norm;
        let z = quat.z / norm;

        let cos = w * 2.0 * w - 1.0;
        let sin = 2.0 * w * z;
        Self {
            a: Vec2::new(cos, sin),
        }
    }

    /// Retrieve the rotor components
    pub fn to_cos_sin(&self) -> (f32, f32) {
        (self.a.x, self.a.y)
    }

    /// Convert the rotor to a quaternion (rotation around Z axis).
    pub fn to_quaternion(&self) -> crate::Quat {
        let angle = self.angle();
        crate::Quat::from_rotation_z(angle)
    }
}

impl Mul<Rotor2d> for Rotor2d {
    type Output = Rotor2d;

    /// Compose two rotors.
    ///
    /// `r1 * r2` applies `r2` first, then `r1`.
    fn mul(self, rhs: Rotor2d) -> Self::Output {
        Self::Output {
            a: Vec2::new(
                self.a.x * rhs.a.x - self.a.y * rhs.a.y,
                self.a.x * rhs.a.y + self.a.y * rhs.a.x,
            ),
        }
    }
}

impl Mul<&Rotor2d> for Rotor2d {
    type Output = Rotor2d;

    /// Compose two rotors.
    /// `r1 * r2` applies `r2` first, then `r1`.
    fn mul(self, rhs: &Rotor2d) -> Self::Output {
        self * *rhs
    }
}

impl Mul<Rotor2d> for &Rotor2d {
    type Output = Rotor2d;

    /// Compose two rotors.
    /// `r1 * r2` applies `r2` first, then `r1`.
    fn mul(self, rhs: Rotor2d) -> Self::Output {
        *self * rhs
    }
}

impl Mul<&Rotor2d> for &Rotor2d {
    type Output = Rotor2d;

    /// Compose two rotors.
    /// `r1 * r2` applies `r2` first, then `r1`.
    fn mul(self, rhs: &Rotor2d) -> Self::Output {
        *self * *rhs
    }
}

impl Mul<Vec2> for Rotor2d {
    type Output = Vec2;

    /// Rotate a vector by the rotor.
    fn mul(self, rhs: Vec2) -> Self::Output {
        Self::Output {
            x: self.a.x * rhs.x - self.a.y * rhs.y,
            y: self.a.x * rhs.y + self.a.y * rhs.x,
        }
    }
}

impl Mul<&Vec2> for Rotor2d {
    type Output = Vec2;

    /// Rotate a vector by the rotor.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        self * *rhs
    }
}

impl Mul<Vec2> for &Rotor2d {
    type Output = Vec2;

    /// Rotate a vector by the rotor.
    fn mul(self, rhs: Vec2) -> Self::Output {
        *self * rhs
    }
}

impl Mul<&Vec2> for &Rotor2d {
    type Output = Vec2;

    /// Rotate a vector by the rotor.
    fn mul(self, rhs: &Vec2) -> Self::Output {
        *self * *rhs
    }
}

#[cfg(test)]
mod tests {
    use approx::assert_relative_eq;

    use super::*;
    use crate::Quat;

    #[test_log::test]
    fn test_rotor2d_x_0deg() {
        let rotor = Rotor2d::radians(0.0);
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, Vec2::X);
    }

    #[test_log::test]
    fn test_rotor2d_x_90deg() {
        let rotor = Rotor2d::radians(core::f32::consts::FRAC_PI_2);
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, Vec2::Y);
    }

    #[test_log::test]
    fn test_rotor2d_x_180deg() {
        let rotor = Rotor2d::radians(core::f32::consts::PI);
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, -Vec2::X);
    }

    #[test_log::test]
    fn test_rotor2d_x_270deg() {
        let rotor = Rotor2d::radians(core::f32::consts::PI + core::f32::consts::FRAC_PI_2);
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, -Vec2::Y);
    }

    #[test_log::test]
    fn test_rotor2d_x_0deg_inverse() {
        let rotor = Rotor2d::radians(0.0);
        let rotor = rotor.inverse();
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, Vec2::X);
    }

    #[test_log::test]
    fn test_rotor2d_x_90deg_inverse() {
        let rotor = Rotor2d::radians(core::f32::consts::FRAC_PI_2);
        let rotor = rotor.inverse();
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, -Vec2::Y);
    }

    #[test_log::test]
    fn test_rotor2d_x_180deg_inverse() {
        let rotor = Rotor2d::radians(core::f32::consts::PI);
        let rotor = rotor.inverse();
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, -Vec2::X);
    }

    #[test_log::test]
    fn test_rotor2d_x_270deg_inverse() {
        let rotor = Rotor2d::radians(core::f32::consts::PI + core::f32::consts::FRAC_PI_2);
        let rotor = rotor.inverse();
        let point = Vec2::X;
        let rotated = rotor * point;
        assert_relative_eq!(rotated, Vec2::Y);
    }

    #[test_log::test]
    fn test_rotor2d_x1_y1_90deg() {
        let rotor = Rotor2d::radians(core::f32::consts::FRAC_PI_2);
        let point = Vec2::new(1.0, 1.0);
        let rotated = rotor * point;
        assert_relative_eq!(rotated, Vec2::new(-1.0, 1.0));
    }

    #[test_log::test]
    fn test_from_quaternion() {
        for angle in 0..360 {
            let angle = angle as f32 * core::f32::consts::PI / 180.0;
            let rotor = Rotor2d::radians(angle);
            let quat = Quat::from_rotation_z(angle);
            let rotor2 = Rotor2d::from_quaternion(quat);
            // assert_eq!(rotor, rotor2);
            // Use a slightly larger epsilon to account for floating-point rounding differences
            assert_relative_eq!(rotor.a, rotor2.a, epsilon = 1e-6);
        }
    }

    #[test_log::test]
    fn test_mul_consistency() {
        let r1 = Rotor2d::radians(core::f32::consts::FRAC_PI_2);
        let r2 = Rotor2d::radians(core::f32::consts::FRAC_PI_4);
        let v = Vec2::new(1.0, 1.0);

        // * for compose is consistent with apply
        assert_relative_eq!(r1 * r2 * v, r1 * (r2 * v));
        assert_relative_eq!((&r1) * r2 * v, r1 * (r2 * v));
        assert_relative_eq!(r1 * (&r2) * v, r1 * (r2 * v));
        assert_relative_eq!((&r1) * (&r2) * v, r1 * (r2 * v));
    }
}

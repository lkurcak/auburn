use crate::Vec3;
use crate::algorithm::{epa, gjk};
use crate::collider::Collider;
use crate::property::support::Support;
use crate::relation::collides::Collides;
use crate::relation::penetrates::Penetrates;
use crate::shape::Shape;
use crate::transformation::Transformation3d;

const GJK_MAX_ITERATIONS: usize = 30;

/// Generic collision detection for any two shapes that implement Support
impl<'a, 'b, S1, S2, T1, T2> Collides<Collider<'b, S2, T2>> for Collider<'a, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation3d,
    T2: Transformation3d,
{
    fn collides(&self, other: &Collider<'b, S2, T2>) -> bool {
        gjk(
            |dir| self.support(dir),
            |dir| other.support(dir),
            GJK_MAX_ITERATIONS,
        )
        .is_some()
    }
}

/// Generic penetration calculation for any two shapes that implement Support
impl<'a, 'b, S1, S2, T1, T2> Penetrates<Collider<'b, S2, T2>> for Collider<'a, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation3d,
    T2: Transformation3d,
{
    fn penetrates(&self, other: &Collider<'b, S2, T2>) -> Option<Vec3> {
        let simplex = gjk(
            |dir| self.support(dir),
            |dir| other.support(dir),
            GJK_MAX_ITERATIONS,
        )?;

        let penetration = epa(simplex, |dir| self.support(dir), |dir| other.support(dir));
        Some(-penetration)
    }
}

// NOTE: Generic TOI implementations are not possible without specialization.
// The specific Vec3 implementations provide optimized TOI for common cases.
// For other transformation types, use the conservative_advancement_toi function directly.

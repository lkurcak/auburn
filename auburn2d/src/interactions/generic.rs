use crate::Vec2;
use crate::algorithm::{epa, gjk};
use crate::collider::Collider;
use crate::property::support::Support;
use crate::relation::collides::Collides;
use crate::relation::penetrates::Penetrates;
use crate::shape::Shape;
use crate::transformation::Transformation2d;

const GJK_MAX_ITERATIONS: usize = 30;

/// Generic collision detection for any two shapes that implement Support
impl<'a, 'b, S1, S2, T1, T2> Collides<Collider<'b, S2, T2>> for Collider<'a, S1, T1>
where
    S1: Shape + Support,
    S2: Shape + Support,
    T1: Transformation2d,
    T2: Transformation2d,
{
    fn collides(&self, other: &Collider<'b, S2, T2>) -> bool {
        // Run GJK using the colliders' support functions (which handle local-to-world transformation)
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
    T1: Transformation2d,
    T2: Transformation2d,
{
    fn penetrates(&self, other: &Collider<'b, S2, T2>) -> Option<Vec2> {
        // Run GJK first to check for collision using the colliders' support functions
        let simplex = gjk(
            |dir| self.support(dir),
            |dir| other.support(dir),
            GJK_MAX_ITERATIONS,
        )?;

        // If collision detected, run EPA to get penetration depth
        let penetration = epa(simplex, |dir| self.support(dir), |dir| other.support(dir));

        // EPA returns the vector to push B away from A, but the convention
        // seems to be that penetrates() should return the vector to push self away from other
        // So we negate it
        Some(-penetration)
    }
}

// NOTE: Generic TOI implementations are not possible without specialization (unstable feature).
// The specific Vec2 implementations provide optimized TOI for common cases.
// For other transformation types, TOI would need to be implemented separately or use
// the conservative_advancement_toi function from the algorithm module directly.

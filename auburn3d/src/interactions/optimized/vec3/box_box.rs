use crate::Vec3;
use crate::algorithm::{
    conservative_advancement_toi, conservative_advancement_toiae,
    conservative_advancement_toi_unbounded, conservative_advancement_toiae_unbounded,
};
use crate::collider::Collider;
use crate::property::support::Support;
use crate::relation::cast::{
    TimeOfImpact, TimeOfImpactAndExit, TimeTravelingTimeOfImpact, TimeTravelingTimeOfImpactAndExit,
};
use crate::shape::box3d::Box3d;

impl TimeOfImpact<Collider<'_, Box3d, Vec3>> for Collider<'_, Box3d, Vec3> {
    fn toi(&self, other: &Collider<'_, Box3d, Vec3>, vel: Vec3) -> Option<f32> {
        conservative_advancement_toi(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

impl TimeOfImpactAndExit<Collider<'_, Box3d, Vec3>> for Collider<'_, Box3d, Vec3> {
    fn toiae(&self, other: &Collider<'_, Box3d, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        conservative_advancement_toiae(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Box3d, Vec3>> for Collider<'_, Box3d, Vec3> {
    fn tttoi(&self, other: &Collider<'_, Box3d, Vec3>, vel: Vec3) -> Option<f32> {
        conservative_advancement_toi_unbounded(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Box3d, Vec3>> for Collider<'_, Box3d, Vec3> {
    fn tttoiae(&self, other: &Collider<'_, Box3d, Vec3>, vel: Vec3) -> Option<(f32, f32)> {
        conservative_advancement_toiae_unbounded(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

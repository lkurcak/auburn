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
use crate::shape::{ball::Ball, point::Point};
use crate::transformation::Transform3d;

impl TimeOfImpact<Collider<'_, Point, Transform3d>> for Collider<'_, Ball, Transform3d> {
    fn toi(&self, other: &Collider<'_, Point, Transform3d>, vel: Vec3) -> Option<f32> {
        conservative_advancement_toi(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

impl TimeOfImpactAndExit<Collider<'_, Point, Transform3d>> for Collider<'_, Ball, Transform3d> {
    fn toiae(&self, other: &Collider<'_, Point, Transform3d>, vel: Vec3) -> Option<(f32, f32)> {
        conservative_advancement_toiae(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Point, Transform3d>>
    for Collider<'_, Ball, Transform3d>
{
    fn tttoi(&self, other: &Collider<'_, Point, Transform3d>, vel: Vec3) -> Option<f32> {
        conservative_advancement_toi_unbounded(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Point, Transform3d>>
    for Collider<'_, Ball, Transform3d>
{
    fn tttoiae(
        &self,
        other: &Collider<'_, Point, Transform3d>,
        vel: Vec3,
    ) -> Option<(f32, f32)> {
        conservative_advancement_toiae_unbounded(
            |dir| self.support(dir),
            |dir| other.support(dir),
            vel,
        )
    }
}

// Reverse
impl TimeOfImpact<Collider<'_, Ball, Transform3d>> for Collider<'_, Point, Transform3d> {
    fn toi(&self, other: &Collider<'_, Ball, Transform3d>, vel: Vec3) -> Option<f32> {
        other.toi(self, -vel)
    }
}

impl TimeOfImpactAndExit<Collider<'_, Ball, Transform3d>> for Collider<'_, Point, Transform3d> {
    fn toiae(&self, other: &Collider<'_, Ball, Transform3d>, vel: Vec3) -> Option<(f32, f32)> {
        other.toiae(self, -vel).map(|(exit, entry)| (entry, exit))
    }
}

impl TimeTravelingTimeOfImpact<Collider<'_, Ball, Transform3d>>
    for Collider<'_, Point, Transform3d>
{
    fn tttoi(&self, other: &Collider<'_, Ball, Transform3d>, vel: Vec3) -> Option<f32> {
        other.tttoi(self, -vel)
    }
}

impl TimeTravelingTimeOfImpactAndExit<Collider<'_, Ball, Transform3d>>
    for Collider<'_, Point, Transform3d>
{
    fn tttoiae(
        &self,
        other: &Collider<'_, Ball, Transform3d>,
        vel: Vec3,
    ) -> Option<(f32, f32)> {
        other.tttoiae(self, -vel).map(|(exit, entry)| (entry, exit))
    }
}

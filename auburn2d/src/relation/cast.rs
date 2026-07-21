use crate::Vec2;

pub trait TimeOfImpact<T> {
    /// Casts the object in the direction of the given vector until it collides with another
    /// object.
    ///
    /// Returns the "time of impact", which can be multiplied by [`vel`] to get the distance
    /// traveled.
    ///
    /// See also [`TimeOfImpactAndExit::toiae`].
    /// See also [`TimeTravelingTimeOfImpact::tttoi`].
    /// See also [`TimeTravelingTimeOfImpactAndExit::tttoiae`].
    fn toi(&self, other: &T, vel: Vec2) -> Option<f32>;
}

pub trait TimeTravelingTimeOfImpact<T> {
    /// Casts the object in the direction of the given vector until it collides with another
    /// object.
    ///
    /// Returns the "time of impact", which can be multiplied by [`vel`] to get the distance
    /// traveled.
    ///
    /// As opposed to [`TimeOfImpact::toi`], time of impact can be negative.
    ///
    /// For more information see [`TimeOfImpact::toi`].
    fn tttoi(&self, other: &T, vel: Vec2) -> Option<f32>;
}

pub trait TimeOfImpactAndExit<T> {
    /// Casts the object in the direction of the given vector.
    ///
    /// Returns the "time of impact" and "time of exit", which can be multiplied by [`vel`] to get the distance
    /// traveled.
    ///
    /// For more information see [`TimeOfImpact::toi`].
    fn toiae(&self, other: &T, vel: Vec2) -> Option<(f32, f32)>;
}

pub trait TimeTravelingTimeOfImpactAndExit<T> {
    /// Casts the object in the direction of the given vector.
    ///
    /// Returns the "time of impact" and "time of exit", which can be multiplied by [`vel`] to get the distance
    /// traveled.
    ///
    /// As opposed to [`TimeOfImpactAndExit::toiae`], times can be negative.
    ///
    /// For more information see [`TimeOfImpact::toi`].
    fn tttoiae(&self, other: &T, vel: Vec2) -> Option<(f32, f32)>;
}

/*
#[derive(Default, Debug, Clone)]
pub struct CastResult {
    /// Time of impact
    pub toi: f32,
    /// Time of exit
    pub toe: f32,
    /// Normal of the impact
    pub noi: Vec2,
    /// Normal of the exit
    pub noe: Vec2,
    /// Point of impact
    pub poi: Vec2,
    /// Point of exit
    pub poe: Vec2,
}

pub trait Cast<T> {
    /// Casts the object in the direction of the given vector until it collides with another
    /// object.
    ///
    /// Returns the "time of impact", which can be multiplied by [`vel`] to get the distance
    /// traveled.
    ///
    /// For more information see [`TimeOfImpact::toi`].
    fn cast(&self, other: &T, vel: Vec2) -> Option<CastResult>;
}
*/

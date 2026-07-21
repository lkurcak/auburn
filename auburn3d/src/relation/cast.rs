use crate::Vec3;

pub trait TimeOfImpact<T> {
    fn toi(&self, other: &T, vel: Vec3) -> Option<f32>;
}

pub trait TimeTravelingTimeOfImpact<T> {
    fn tttoi(&self, other: &T, vel: Vec3) -> Option<f32>;
}

pub trait TimeOfImpactAndExit<T> {
    fn toiae(&self, other: &T, vel: Vec3) -> Option<(f32, f32)>;
}

pub trait TimeTravelingTimeOfImpactAndExit<T> {
    fn tttoiae(&self, other: &T, vel: Vec3) -> Option<(f32, f32)>;
}

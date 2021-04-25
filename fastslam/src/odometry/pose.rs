use crate::geometry::point::Point;
use crate::math::scalar::Angle;
use crate::geometry::Vector;

#[derive(Debug, Clone)]
pub struct Pose {
    pub position: Vector,
    pub heading: Angle,
}

impl Default for Pose {
    fn default() -> Pose {
        Pose::new(Vector::new(1.0, 1.0), 0.2)
    }
}

impl Pose {
    pub fn new(position: Vector, heading: Angle) -> Pose {
        Pose { position, heading }
    }
}
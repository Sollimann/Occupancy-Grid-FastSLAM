use crate::geometry::point::Point;
use crate::math::scalar::Angle;

#[derive(Debug, Clone)]
pub struct Pose {
    pub position: Point,
    pub heading: Angle,
}

impl Default for Pose {
    fn default() -> Pose {
        Pose::new(Point::new(0.0, 0.0), 0.0)
    }
}

impl Pose {
    pub fn new(position: Point, heading: Angle) -> Pose {
        Pose { position, heading }
    }
}
use crate::geometry::point::Point;
use crate::math::scalar::Angle;
use std::ops;
use piston_window::Position;

#[derive(Debug, Clone)]
pub struct Pose {
    pub position: Point,
    pub heading: Angle,
}

/// overload Vector addition
impl ops::Add for Pose {
    type Output = Pose;

    fn add(self, rhs: Self) -> Self::Output {
        let position = self.position + rhs.position;
        let heading = self.heading + rhs.heading;
        Pose::new(position, heading)
    }
}

/// overload Vector addition
impl ops::AddAssign for Pose {
    fn add_assign(&mut self, rhs: Self) {
        *self = Self {
            position: self.position + rhs.position,
            heading: self.heading + rhs.heading
        }
    }
}

impl Default for Pose {
    fn default() -> Pose {
        Pose::new(Point::new(1.0, 1.0), 0.2)
    }
}

impl Pose {
    pub fn new(position: Point, heading: Angle) -> Pose {
        Pose { position, heading }
    }
}
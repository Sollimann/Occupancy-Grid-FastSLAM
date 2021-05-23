use crate::geometry::point::Point;
use crate::math::scalar::{Angle, Scalar};
use std::{ops, cmp, fmt};
use approx::{RelativeEq};

#[derive(Debug, Clone)]
pub struct Pose {
    pub position: Point,
    pub heading: Angle,
}

/// decide how a Point should be displayed when formatting and printing
impl fmt::Display for Pose {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Pose(x: {}, y: {}, theta: {})", self.position.x, self.position.y, self.heading)
    }
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

/// define partial equality for two Point's
impl cmp::PartialEq for Pose {
    fn eq(&self, other: &Self) -> bool {

        let pose = self.position == other.position;
        let theta = self.heading == other.heading;

        pose && theta
    }
}

/// overload Pose multiplied by Scalar
impl ops::Mul<Scalar> for Pose {
    type Output = Pose;

    fn mul(self, s: Scalar) -> Self::Output {

        Pose::new(self.position * s, self.heading * s)
    }
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
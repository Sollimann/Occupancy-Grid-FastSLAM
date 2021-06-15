use crate::geometry::point::Point;
use crate::math::scalar::{Angle, Scalar};
use std::{ops, cmp, fmt};
use approx::{RelativeEq};
use std::ops::{Sub, Mul};

#[derive(Debug, Clone, Copy)]
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

/// overload Pose addition
impl ops::Add for Pose {
    type Output = Pose;

    fn add(self, rhs: Self) -> Self::Output {
        let position = self.position + rhs.position;
        let heading = self.heading + rhs.heading;
        Pose::new(position, heading)
    }
}

/// overload Pose subtraction
impl ops::Sub for Pose {
    type Output = Pose;

    fn sub(self, rhs: Self) -> Self::Output {
        let position = self.position - rhs.position;
        let heading = self.heading - rhs.heading;
        Pose::new(position, heading)
    }
}

/// overload Pose multiplication
impl ops::Mul for Pose {
    type Output = Pose;

    fn mul(self, rhs: Self) -> Self::Output {
        let position = self.position * rhs.position;
        let heading = self.heading * rhs.heading;
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


/// overload Pose divided by Scalar
impl ops::Div<Scalar> for Pose {
    type Output = Pose;

    fn div(self, s: Scalar) -> Self::Output {
        Pose::new(self.position / s, self.heading / s)
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
    
    pub fn sqrt(&self) -> Pose {
        let x_sqrt = self.position.x.sqrt();
        let y_sqrt = self.position.y.sqrt();
        let heading_sqrt = self.heading.sqrt();
        Pose::new(Point::new(x_sqrt, y_sqrt), heading_sqrt)
    }
}
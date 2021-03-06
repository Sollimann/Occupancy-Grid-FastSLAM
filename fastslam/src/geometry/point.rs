use std::fmt;
use std::cmp;
use std::ops;

use crate::math::scalar::{Scalar, Angle};
use crate::geometry::vector::Vector;
use std::ops::{Add, Div, Mul};

#[derive(Debug, Copy, Clone)]
pub struct Point {
    pub x: Scalar,
    pub y: Scalar
}

/// decide how a Point should be displayed when formatting and printing
impl fmt::Display for Point {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Point({}, {})", self.x, self.y)
    }
}

/// define partial equality for two Point's
impl cmp::PartialEq for Point {
    fn eq(&self, other: &Self) -> bool {
        self.x == other.x && self.y == other.y
    }
}

/// overload Point addition
impl ops::Add for Point {
    type Output = Point;

    fn add(self, rhs: Self) -> Self::Output {
        Point::new(self.x + rhs.x, self.y + rhs.y)
    }
}

/// overload Point subtraction
impl ops::Sub for Point {
    type Output = Point;

    fn sub(self, rhs: Self) -> Self::Output {
        Point::new(self.x - rhs.x, self.y - rhs.y)
    }
}

/// overload Point + Vector = Point addition
impl ops::Add<Vector> for Point {
    type Output = Point;

    fn add(self, vec: Vector) -> Self::Output {
        Point::new(self.x + vec.x, self.y + vec.y)
    }
}

/// overload Point - Vector = Point subtraction
impl ops::Sub<Vector> for Point {
    type Output = Point;

    fn sub(self, vec: Vector) -> Self::Output {
        Point::new(self.x - vec.x, self.y - vec.y)
    }
}

/// overload Point multiplied by Scalar
impl ops::Mul<Scalar> for Point {
    type Output = Point;

    fn mul(self, s: Scalar) -> Self::Output {
        Point::new(self.x * s, self.y * s)
    }
}

/// overload Point multiplied by Point
impl ops::Mul<Point> for Point {
    type Output = Point;

    fn mul(self, rhs: Point) -> Self::Output {
        Point::new(self.x * rhs.x, self.y * rhs.y)
    }
}

/// overload Point divided by Scalar
impl ops::Div<Scalar> for Point {
    type Output = Point;

    fn div(self, s: Scalar) -> Self::Output {
        Point::new(self.x / s, self.y / s)
    }
}

impl Point {
    pub fn new(x: Scalar, y: Scalar) -> Point {
        Point {x, y}
    }

    pub fn to_point_vec(&self, p: Point) -> Vector {
        let x = p.x - self.x;
        let y = p.y - self.y;
        Vector::new(x, y)
    }

    pub fn to_vec(&self) -> Vector {
        Vector::new(self.x, self.y)
    }

    pub fn dist_to_point(&self, p: Point) -> Scalar {
        self.to_point_vec(p).length()
    }

    pub fn angle_to_point(&self, p: Point) -> Angle {
        let vec = self.to_point_vec(p);
        vec.angle()
    }
}
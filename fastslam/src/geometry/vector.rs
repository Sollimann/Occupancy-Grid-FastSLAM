use std::cmp;
use std::fmt;
use std::ops;

use crate::math::scalar::{Angle, Scalar};
use std::fmt::Formatter;
use std::ops::{Add, Sub, Mul};

#[derive(Debug, Clone, Copy)]
pub struct Vector {
    pub x: Scalar,
    pub y: Scalar,
}

pub type Mat2 = [[f64; 2]; 2];

/// decide how a Vector should be displayed when formatting and printing
impl fmt::Display for Vector {
    fn fmt(&self, f: &mut Formatter) -> fmt::Result {
        write!(f, "({}, {})", self.x, self.y)
    }
}

/// overload Vector addition
impl ops::Add for Vector {
    type Output = Vector;

    fn add(self, rhs: Self) -> Self::Output {
        Vector::new(self.x + rhs.x, self.y + rhs.y)
    }
}

/// overload Vector subtraction
impl ops::Sub for Vector {
    type Output = Vector;

    fn sub(self, rhs: Self) -> Self::Output {
        Vector::new(self.x - rhs.x, self.y - rhs.y)
    }
}

/// overload Vector multiplied by Scalar
impl ops::Mul<Scalar> for Vector {
    type Output = Vector;

    fn mul(self, s: Scalar) -> Self::Output {
        Vector::new(self.x * s, self.y * s)
    }
}

/// define partial equality for two Vector's
impl cmp::PartialEq for Vector {
    fn eq(&self, other: &Vector) -> bool {
        self.x == other.x && self.y == other.y
    }
}

impl Vector {

    /// create a new Vector
    pub fn new(x: Scalar, y: Scalar) -> Self {
        Vector {x, y}
    }

    /// return the unit vector for a given angle
    pub fn from_angle(rad: Angle) -> Vector {
        // 0° is in forward direction (along X-axis)
        // 90° is to the left (along Y-axis)
        Vector {
            x: rad.cos(),
            y: rad.sin(),
        }
    }

    /// calculate the length of the Vector
    pub fn length(&self) -> Scalar {
        (self.x.powi(2) + self.y.powi(2)).sqrt()
    }

    /// calculate dot product between two Vector's
    pub fn dot(&self, q: Vector) -> Scalar {
        self.x * q.x + q.y * self.y
    }

    /// calculate product of vector and a vector transpose
    pub fn prod(&self, q: Vector) -> Mat2 {
        let (tx, ty) = (self.x, self.y);
        [[tx* q.x, ty * q.y], [ty* q.x, ty * q.y]]
    }

    /// calculate cross product of two Vector's. The result is the Resultant
    /// which is the length of the vector perpendicular (orthogonal) to both vectors
    pub fn cross(&self, q: Vector) -> Scalar {
        self.x * q.y - q.x * self.y
    }

    /// Vector angle relative to 0° in forward direction (along X-axis)
    pub fn angle(&self) -> Angle {
        self.y.atan2(self.x)
    }

    /// rotate a Vector by angle
    pub fn rotate(&self, angle: Angle) -> Vector {
        let c = angle.cos();
        let s = angle.sin();
        Vector::new(c * self.x - s * self.y, s * self.x + c * self.y)
    }
}
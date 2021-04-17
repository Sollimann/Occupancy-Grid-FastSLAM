use crate::geometry::vector::Vector;
use crate::math::scalar::Angle;
use crate::geometry::point::Point;

pub struct Ray {
    pub origin: Point,
    pub direction: Vector,
}

impl Ray {
    pub fn new(origin: Point, direction: Vector) -> Ray {
        Ray { origin, direction }
    }

    pub fn from_angle(origin: Point, angle: Angle) -> Ray {
        Ray {
            origin,
            direction: Vector::from_angle(angle),
        }
    }
}
use crate::geometry::vector::Vector;
use crate::math::scalar::Angle;

#[derive(Debug, Clone)]
pub struct Twist {
    pub velocity: Vector,
    pub angular: Angle,
}

impl Default for Twist {
    fn default() -> Twist {
        Twist::new(Vector::new(0.0, 0.0), 0.0)
    }
}

impl Twist {
    pub fn new(velocity: Vector, angular: Angle) -> Twist {
        Twist { velocity, angular }
    }
}
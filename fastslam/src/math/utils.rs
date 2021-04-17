use crate::geometry::vector::Vector;
use crate::math::scalar::Scalar;

pub fn compute_vector_abs_diff(u: Vector, v: Vector) -> (f64, f64) {
    let abs_difference_x = (u.x - v.x).abs();
    let abs_difference_y = (u.y - v.y).abs();
    (abs_difference_x, abs_difference_y)
}

pub fn compute_abs_diff(actual: Scalar, compared: Scalar) -> f64 {
    (actual - compared).abs()
}
use crate::geometry::ray::Ray;
use crate::geometry::point::Point;

pub trait Target {
    fn intersect(&self, ray: &Ray) -> Vec<Point>;
}
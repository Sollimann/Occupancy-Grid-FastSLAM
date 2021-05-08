use std::fmt;
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator};
use rayon::slice;
use crate::geometry::point::Point;
use crate::math::scalar::Scalar;


#[derive(Debug, Clone)]
pub struct PointCloud {
    points: Vec<Point>,
}

/// decide how a Point should be displayed when formatting and printing
impl fmt::Display for PointCloud {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "Pointcloud size: ({})", self.size())
    }
}

impl PointCloud {
    pub fn new(points: Vec<Point>) -> PointCloud {
        PointCloud { points }
    }

    pub fn empty() -> PointCloud {
        PointCloud { points: vec![] }
    }

    pub fn size(&self) -> usize {
        self.points.len()
    }

    pub fn add(&mut self, p: Point) {
        self.points.push(p);
    }

    pub fn iter(&self) -> std::slice::Iter<Point> {
        self.points.iter()
    }

    pub fn par_iter(&self) -> rayon::slice::Iter<Point> {
        self.points.par_iter()
    }

    pub fn iter_mut(&mut self) -> rayon::slice::IterMut<Point> {
        self.points.par_iter_mut()
    }

    pub fn centroid(&self) -> Point {
        let x_avg: Scalar = self.iter().map(|p: &Point| p.x).sum::<Scalar>() / self.size() as Scalar;
        let y_avg: Scalar = self.iter().map(|p: &Point| p.y).sum::<Scalar>() / self.size() as Scalar;
        Point::new(x_avg, y_avg)
    }
}

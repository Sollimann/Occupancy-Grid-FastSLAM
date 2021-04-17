use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator};
use rayon::slice::{Iter, IterMut};
use crate::geometry::point::Point;

pub struct PointCloud {
    points: Vec<Point>,
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

    pub fn iter(&self) -> Iter<Point> {
        self.points.par_iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<Point> {
        self.points.par_iter_mut()
    }
}

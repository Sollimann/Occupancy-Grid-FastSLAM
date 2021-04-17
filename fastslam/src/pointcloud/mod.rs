use std::fmt;
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator};
use rayon::slice::{Iter, IterMut};
use crate::geometry::point::Point;

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

    pub fn iter(&self) -> Iter<Point> {
        self.points.par_iter()
    }

    pub fn iter_mut(&mut self) -> IterMut<Point> {
        self.points.par_iter_mut()
    }
}

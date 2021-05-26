use crate::geometry::point::Point;
use crate::geometry::vector::Vector;
use crate::math::scalar::{Angle, Scalar};
use crate::odometry::pose::Pose;
use crate::pointcloud::PointCloud;
use std::slice::Iter;
use std::iter::FromIterator;

/// A single measurement (distance reading) of a laser scanner.
#[derive(Debug, Clone, Copy)]
pub struct Measurement {
    pub angle: Angle,
    pub distance: Scalar,
}

impl Measurement {
    pub fn new(angle: Angle, distance: Scalar) -> Measurement {
        Measurement { angle, distance }
    }

    pub fn to_point(&self, pose: &Pose) -> Point {
        let p0 = pose.position;
        let direction = Vector::from_angle(pose.heading + self.angle);
        let p1 = p0 + direction * self.distance;
        p1
    }
}

/// A full 360° scan from a laser scanner.
#[derive(Debug, Clone)]
pub struct Scan {
    pub measurements: Vec<Measurement>,
}

// needed to be able to iter().collect() on scans
impl FromIterator<Measurement> for Scan {
    fn from_iter<T: IntoIterator<Item=Measurement>>(iter: T) -> Self {
        let mut scan = Scan::empty();

        for meas in iter {
            scan.add(meas)
        }

        scan
    }
}

impl Scan {
    pub fn empty() -> Scan {
        Scan {
            measurements: Vec::new(),
        }
    }

    pub fn add(&mut self, m: Measurement) {
        self.measurements.push(m);
    }

    pub fn iter(&self) -> Iter<Measurement> {
        self.measurements.iter()
    }

    pub fn to_pointcloud(&self, pose: &Pose) -> PointCloud {
        PointCloud::new(self.iter().map(|m| m.to_point(pose)).collect())
    }
}

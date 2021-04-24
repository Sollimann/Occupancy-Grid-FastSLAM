use std::cmp::Ordering;
use crate::odometry::Pose;
use crate::geometry::{Line, Point, Ray, Target};
use crate::sensor::laserscanner::{Scan, Measurement};
use crate::math::scalar::{Scalar, Angle, PI};

pub struct LaserScanner {
    pub num_columns: u32,
    // max_range: Scalar,
    // range_noise: Scalar,
    // angle_noise: Angle
}

impl LaserScanner {
    pub fn scan(&self, pose: &Pose, targets: &[Line]) -> Scan {
        let mut scan = Scan::empty();

        // comparison function to find distance from robot pose to a laser scan point
        //let distance = |p: &Point| (*p - pose.position).length();
        let distance = |p: &Point| (pose.position.to_point_vec(*p)).length();

        // Raycasting
        for col in 0..self.num_columns {
            let col_angle = self.column_to_angle(col);
            let ray = Ray::from_angle(pose.position, pose.heading + col_angle);

            let mut points = vec![];
            for target in targets.iter() {
                let mut candidates = target.intersect(&ray);
                points.append(&mut candidates)
            }

            // only take the point closest to the sensor (first point of collision)
            let closest = points.iter().min_by(|p1, p2| {
                distance(p1)
                    .partial_cmp(&distance(p2))
                    .unwrap_or(Ordering::Equal)
            });

            if let Some(p) = closest {
                scan.add(Measurement::new(col_angle, distance(p)))
            }
        }

        scan
    }

    // current range: 0 - 2*PI, might want this to be range (-PI, PI)
    fn column_to_angle(&self, column: u32) -> Angle {
        Scalar::from(column) / Scalar::from(self.num_columns) * 2.0 * PI
    }
}

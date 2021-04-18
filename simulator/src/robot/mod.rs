use fastslam::odometry::odometry::Odometry;
use crate::sensor::laserscanner::LaserScanner;

pub struct Robot {
    pub odom: Odometry,
    pub laser_scanner: LaserScanner,
}
use crate::odometry::Odometry;
use crate::simulator::laserscanner::LaserScanner;

pub struct Robot {
    pub odom: Odometry,
    pub laser_scanner: LaserScanner,
}
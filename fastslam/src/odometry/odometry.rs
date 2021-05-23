use crate::geometry::point::Point;
use crate::odometry::pose::Pose;
use crate::odometry::twist::Twist;

#[derive(Debug, Clone)]
pub struct Odometry {
    pub pose: Pose,
    pub vel: Twist,
}

impl Default for Odometry {
    fn default() -> Odometry {
        Odometry::new(
            Pose::default(),
            Twist::default()
        )
    }
}

impl Odometry {
    pub fn new(pose: Pose, vel: Twist) -> Odometry {
        Odometry { pose, vel }
    }
}
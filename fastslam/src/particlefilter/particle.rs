use crate::odometry::Pose;
use crate::gridmap::grid_map::GridMap;
use crate::math::scalar::Scalar;
use crate::sensor::laserscanner::Scan;

pub struct Particle {
    pub pose: Pose,
    pub weight: Scalar,
    pub gridmap: GridMap
}

// TODO: remove this later
impl Default for Particle {
    fn default() -> Self {
        Particle {
            pose: Pose::default(),
            weight: 1.0,
            gridmap: GridMap::default()
        }
    }
}

impl Particle {
    pub fn new(pose: Pose, weight: Scalar, gridmap: GridMap) -> Self {
        Particle {
            pose,
            weight,
            gridmap
        }
    }

    pub fn cycle(&mut self, scan: &Scan, pose: &Pose) {

        // TODO: this is cheating
        self.pose = pose.clone();

        self.gridmap.update(&self.pose, scan);
    }
}
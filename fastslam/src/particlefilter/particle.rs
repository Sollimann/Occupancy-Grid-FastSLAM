use crate::odometry::Pose;
use crate::gridmap::grid_map::GridMap;
use crate::math::scalar::Scalar;
use crate::sensor::laserscanner::Scan;

#[derive(Debug, Clone)]
pub struct Particle {
    pub pose: Pose, // particle's pose (x, y, thetha)
    pub weight: Scalar, // particle's current weight
    pub gridmap: GridMap // particle's estimated grid map of the environment
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
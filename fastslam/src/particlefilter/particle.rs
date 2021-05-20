use crate::odometry::Pose;
use crate::gridmap::grid_map::GridMap;
use crate::math::scalar::Scalar;
use crate::sensor::laserscanner::Scan;
use crate::pointcloud::PointCloud;


#[derive(Debug, Clone)]
pub struct Particle {
    pub prev_pointcloud: PointCloud,
    pub pose: Pose, // particle's pose (x, y, theta)
    pub weight: Scalar, // particle's current weight
    pub gridmap: GridMap // particle's estimated grid map of the environment
}

// TODO: remove this later
impl Default for Particle {
    fn default() -> Self {
        Particle {
            prev_pointcloud: PointCloud::empty(),
            pose: Pose::default(),
            weight: 1.0,
            gridmap: GridMap::default()
        }
    }
}

impl Particle {
    pub fn new(pose: Pose, weight: Scalar, gridmap: GridMap) -> Self {
        Particle {
            prev_pointcloud: PointCloud::empty(),
            pose,
            weight,
            gridmap
        }
    }

    pub fn get_prev_observation(&self) -> PointCloud {
        self.prev_pointcloud.clone()
    }

    pub fn cycle(&mut self, scan: &Scan, pose: &Pose) {

        // TODO: this is cheating
        self.pose = pose.clone();
        self.gridmap.update(&self.pose, scan);
    }
}
use crate::gridmap::grid_map::GridMap;
use crate::odometry::{Pose};
use crate::sensor::laserscanner::Scan;

pub struct ParticleFilter {
    pub gridmap: GridMap,
    pub pose_estimate: Pose
}

impl Default for ParticleFilter {
    fn default() -> ParticleFilter {
        ParticleFilter {
            gridmap: GridMap::default(),
            pose_estimate: Pose::default()
        }
    }
}

impl ParticleFilter {
    pub fn cycle(&mut self, scan: &Scan, pose: &Pose) {

        // TODO: this is cheating
        self.pose_estimate = pose.clone();

        self.gridmap.update(&self.pose_estimate, scan);
    }
}
use crate::odometry::{Pose, Twist};
use crate::sensor::laserscanner::Scan;
use crate::gridmap::grid_map::GridMap;

/// Computes the motion model probability of a sampled pose.
/// This is the probability p(x_t | x_t-1, u_t) of being at pose x_t after executing
/// control u_t beginning at state x_t-1 assuming that the control is carried out
/// for the fixed duration of dt
///
/// More details: p.123 Table 5.1 in probabilistic robotics, Sebastian Thrun et al.
///
/// Input:
///     curr_particle_pose: the sampled pose after scan match correction
///     prev_particle_pose: the last estimated pose for the particle
///     gain: most recent control input
///     dt: duration of the control gain
/// Returns:
///     p: probability (0.0 - 1.0)
pub fn motion_model_velocity(curr_sampled_pose: &Pose, prev_particle_pose: &Pose, gain: &Twist, dt: f64) -> f64 {
    1.0
}

/// Computes the measurement model probability
/// This is the probability p(z_t | x_j, m_t-1) of measuring z_t and time t, where the robot pose
/// is x_j (sampled pose after scan matching) and m_t-1 is the previous map of the environment
///
/// More details: p.172 Table 6.3 in probabilistic robotics, Sebastian Thrun et al.
///
/// Input:
///     scan: the latest scan
///     curr_sampled_pose: the sampled pose after scan match correction
///     prev_gridmap: the latest gridmap estimate for the particle
/// Returns:
///     p: probability (0.0 - 1.0)
pub fn likelihood_field_range_finder_model(scan: &Scan, curr_sampled_pose: &Pose, prev_gridmap: &GridMap) -> f64 {
    let q = 1.0;
    q
}
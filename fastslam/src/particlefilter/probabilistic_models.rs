use crate::odometry::{Pose, Twist};
use crate::sensor::laserscanner::{Scan, Measurement};
use crate::gridmap::grid_map::GridMap;
use crate::math::scalar::PI;
use crate::geometry::Point;

/// Computes the motion model probability of a sampled pose.
/// This is the probability p(x_t | x_t-1, u_t) of being at pose x_t after executing
/// control u_t beginning at state x_t-1 assuming that the control is carried out
/// for the fixed duration of dt
///
/// More info: 
///  - p.123 Table 5.1 in probabilistic robotics, Sebastian Thrun et al.
///
/// Input:
///     curr_particle_pose: the sampled pose after scan match correction
///     prev_particle_pose: the last estimated pose for the particle
///     gain: most recent control input
///     dt: duration of the control gain
/// Returns:
///     p: probability (0.0 - 1.0)
pub fn motion_model_velocity(curr_sampled_pose: &Pose, prev_particle_pose: &Pose, gain: &Twist, dt: f64) -> f64 {
    // motion noise params
    // alpha_1:2: angular error
    // alpha_3:4: translational error
    let alpha = [0.0015, 0.45, 0.75, 0.75, 0.0, 0.0]; // these values can be tuned

    let v = gain.velocity.x;
    let omega = gain.angular;

    let x_prime = curr_sampled_pose.position.x;
    let y_prime = curr_sampled_pose.position.y;
    let theta_prime = curr_sampled_pose.heading;

    let x = prev_particle_pose.position.x;
    let y = prev_particle_pose.position.y;
    let theta = prev_particle_pose.heading;

    // compute mu
    let mu = 0.5 *
        ((x - x_prime) * theta.cos() + (y - y_prime) * theta.sin()) /
        ((y - y_prime) * theta.cos() - (x - x_prime) * theta.sin());


    let x_star = (x + x_prime) / 2.0 + mu*(y - y_prime);
    let y_star = (y + y_prime) / 2.0 + mu*(x_prime - x);
    let r_star = ((x - x_star).powi(2) + (y - y_star).powi(2)).sqrt();

    let delta_theta = (y_prime - y_star).atan2(x_prime - x_star) - (y - y_star).atan2(x - x_star);

    let v_hat = (delta_theta / dt) * r_star;
    let omega_hat = delta_theta / dt;
    let gamma_hat = ((theta_prime - theta) / dt) - omega_hat;

    let p1 = prob_normal_distribution(v - v_hat, alpha[0]*v.powi(2) + alpha[1] * omega.powi(2));
    let p2 = prob_normal_distribution(omega - omega_hat, alpha[2]*v.powi(2) + alpha[3] * omega.powi(2));
    let p3 = prob_normal_distribution(gamma_hat, alpha[4]*v.powi(2) + alpha[5]*omega.powi(2));
    return p1 * p2 // p1 * p2 * p3
}

/// Computes the measurement model probability
/// This is the probability p(z_t | x_j, m_t-1) of measuring z_t and time t, where the robot pose
/// is x_j (sampled pose after scan matching) and m_t-1 is the previous map of the environment
///
/// More info:
///  - p.172 Table 6.3 in probabilistic robotics, Sebastian Thrun et al.
///  - https://calvinfeng.gitbook.io/probabilistic-robotics/basics/robot-perception/01-beam-models-of-range-finders
///  - https://calvinfeng.gitbook.io/probabilistic-robotics/basics/robot-perception/02-likelihood-fields-for-range-finders
///
/// Input:
///     scan: the latest scan
///     curr_sampled_pose: the sampled pose after scan match correction
///     prev_gridmap: the latest gridmap estimate for the particle
/// Returns:
///     p: probability (0.0 - 1.0)
pub fn likelihood_field_range_finder_model(scan: &Scan, curr_sampled_pose: &Pose, prev_gridmap: &GridMap) -> f64 {
    let z_short = 0.1; // range: (0.1 - 0.3)
    let z_hit = 1.0 - z_short; // range: (0.6-0.9)
    let z_max = 30.0; // maximum allowed sensor value
    let z_rand = 0.05; // random distance noise
    let sigma_hit: f64 = 0.03; //
    let mut q = 1.0;

    // filter out readings equal to or larger than max laser range
    let filtered_scan: Scan = scan
        .iter()
        .filter(|&meas| meas.distance < z_max)
        .cloned()
        .collect();

    // collect all occupied cells in gridmap
    let occupied_cells = prev_gridmap.get_all_occupied_cells();

    // the Euclidean distance between the measurement coordinates (x_z, y_z)
    // and the nearest object in the map m
    filtered_scan.to_pointcloud(curr_sampled_pose).iter().for_each(|z: &Point|{
        let mut min_dist = 9999.0;

        occupied_cells.iter().for_each(|o| {
            let dist = o.to_point_vec(*z).length();
            if dist < min_dist {
                min_dist = dist
            }
        });

        q = q * (z_hit * prob_normal_distribution(min_dist, sigma_hit.powi(2)) + z_rand / z_max)
    });

    return q
}

fn prob_normal_distribution(a: f64, b_squared: f64) -> f64 {
    (1.0 / (2.0 * PI * b_squared).sqrt()) * (-0.5 * a.powi(2) / b_squared).exp()
}
use crate::gridmap::grid_map::GridMap;
use crate::odometry::{Pose, Twist, MotionModel};
use crate::sensor::laserscanner::Scan;
use crate::particlefilter::particle::Particle;
use rayon::prelude::*;
use crate::math::timer::Timer;
use crate::scanmatching::icp::icp;
use crate::geometry::Point;
use crate::sensor::noise::gaussian;
use crate::particlefilter::probabilistic_models::{motion_model_velocity, likelihood_field_range_finder_model};



#[derive(Clone, Debug)]
pub struct ParticleFilter {
    simulation: bool,
    timer: Timer,
    n_particles: usize,
    particles: Vec<Particle>,
    pub gridmap: GridMap,
    pub pose_estimate: Pose
}

impl Default for ParticleFilter {
    fn default() -> ParticleFilter {
        let n_particles: usize = 50;
        let mut particles: Vec<Particle> = vec![];

        // initialize particle list
        let init_gridmap = GridMap::default();
        let init_pose = Pose::default();
        for _ in 0..n_particles {
            particles.push(Particle::new(init_pose.clone(), 1.0, init_gridmap.clone()));
        }

        assert_eq!(particles.len(), n_particles);

        ParticleFilter {
            simulation: true,
            timer: Timer::init_time(),
            n_particles,
            particles: vec![],
            gridmap: init_gridmap,
            pose_estimate: init_pose
        }
    }
}

impl MotionModel for ParticleFilter {}

impl ParticleFilter {

    /// particles: S_t-1 - the sample set of the previous step
    /// scan: z_t - the most recent laser scan
    /// gain: u_t-1 - the most recent gain, applied in the previous step
    pub fn cycle(&mut self, scan: &Scan, gain: &Twist) {

        let dt = if self.simulation {
            1.0 // 1.0s runs nicely with the simulator
        } else {
            self.timer.get_dt()
        };


        // This is an iterator-like chain that potentially executes in parallel
        // we iterate over all particles in the filter and do the following
        self.particles
            .par_iter_mut()
            .for_each(|p: &mut Particle| {

                // step 1.)
                // initial guess of pose x'_ based on motion model
                let motion_model_pose = Self::sample_motion_model_velocity(&p.pose, &gain, dt);

                // step 2.)
                // scan-matching using the initial guess x'_t and the latest scan m_t
                // to compute a pose estimate x*_t
                let curr_pointcloud = scan.to_pointcloud(&p.pose);
                if p.prev_pointcloud.size() == 0 {
                    p.prev_pointcloud = curr_pointcloud.clone();
                }

                let pose_correction = icp(&curr_pointcloud, &p.prev_pointcloud, 20, 0.0001);
                let scan_match_pose = motion_model_pose + pose_correction;

                // step 3.)
                // sample points around the pose x*_t
                let std_dev_sampling = Pose::new(Point::new(1.0, 1.0), 0.05);
                let pose_samples: Vec<Pose> = Self::sample_distribution(&scan_match_pose, std_dev_sampling, 10);

                // step 4.)
                // compute new pose x_t drawn from the gaussian approximation of the
                // improved proposal distribution
                let (improved_pose, eta) = Self::improved_proposal(
                    &pose_samples,
                    &p.pose,
                    &p.gridmap,
                    &scan,
                    &gain,
                    dt
                );

                // step 5.)
                // update the importance weights
                p.weight = p.weight * eta;

                // step 6.)
                // updating the map according to the drawn pose x_t and the observation z_t
                p.gridmap.update(&p.pose, scan)

                // step 7.)
                // compute efficient number of particles and resample based on
                // computed weights if Neff drops below threshold

            });

        println!("particles: {:?} ", self.particles);
    }

    #[allow(non_snake_case)]
    fn sample_distribution(mean: &Pose, std_dev: Pose, K: usize) -> Vec<Pose> {
        let mut samples: Vec<Pose> = Vec::with_capacity(K);
        for _ in 0..K {
            let x = gaussian(mean.position.x, std_dev.position.x);
            let y = gaussian(mean.position.y, std_dev.position.y);
            let theta = gaussian(mean.heading, std_dev.heading);
            let p = Pose::new(Point { x, y }, theta);
            samples.push(p)
        }
        samples
    }

    fn improved_proposal(
        sampled_poses: &Vec<Pose>,
        prev_particle_pose: &Pose,
        prev_gridmap: &GridMap,
        scan: &Scan,
        gain: &Twist,
        dt: f64
    ) -> (Pose, f64) {
        // types are defined at compile-time, so this should not cause overhead
        struct PoseWithDistribution {
            pub pose: Pose,
            pub p_z: f64,
            pub p_x: f64
        }

        let mut mu = Pose::default();
        let mut eta = 0.0;

        let mut poses_with_distribution: Vec<PoseWithDistribution> = vec![];

        sampled_poses.into_iter().for_each(|x_j: &Pose| {
            let p_x = motion_model_velocity(&x_j, prev_particle_pose, gain, dt);
            let p_z = likelihood_field_range_finder_model(scan, &x_j, prev_gridmap);
            mu += *x_j * p_z * p_x;
            eta += p_z * p_x;

            // to avoid re-computing, keep a cache
            poses_with_distribution.push(PoseWithDistribution { pose: *x_j, p_z, p_x });
        });

        // get final estimate of mean pose by normalizing the mean using normalization factor
        mu = mu / eta;

        let mut sigma = Pose::default(); // (0,0,0)

        poses_with_distribution.into_iter().for_each(|pwd: PoseWithDistribution| {
            let (x_j, p_z, p_x) = (pwd.pose, pwd.p_z, pwd.p_x);
            sigma = sigma + (x_j - mu) * (x_j - mu) * p_z * p_x;
        });

        // get final estimate of sigma (pose variance)
        sigma = sigma / eta;

        // sample final particle pose
        let improved_pose = match Self::sample_distribution(&mu, sigma.sqrt(), 1).first() {
            None => panic!("could not sample new pose!"),
            Some(p) => *p
        };

        return (improved_pose, eta)
    }
}
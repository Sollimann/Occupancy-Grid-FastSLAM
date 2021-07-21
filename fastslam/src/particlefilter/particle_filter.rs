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
use crate::particlefilter::resampling::low_variance_sampler;


#[derive(Clone)]
pub struct ParticleFilter {
    simulation: bool,
    timer: Timer,
    n_particles: usize,
    particles: Vec<Particle>,
    pub best_particle: Particle
}

impl Default for ParticleFilter {
    fn default() -> ParticleFilter {
        let n_particles: usize = 20;
        let mut particles: Vec<Particle> = vec![];

        // initialize particle list
        let init_gridmap = GridMap::default();
        let init_pose = Pose::default();
        let init_particle = Particle::new(init_pose.clone(), 1.0, init_gridmap.clone());
        for _ in 0..n_particles {
            particles.push(init_particle.clone());
        }

        assert_eq!(particles.len(), n_particles);

        ParticleFilter {
            simulation: true,
            timer: Timer::init_time(),
            n_particles,
            particles,
            best_particle: init_particle
        }
    }
}

impl MotionModel for ParticleFilter {}

#[allow(non_snake_case)]
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
                let curr_pointcloud = scan.to_pointcloud(&motion_model_pose);
                let scan_match_pose = if p.prev_pointcloud.size() == 0 {
                    motion_model_pose
                } else {
                    let pose_correction = icp(&curr_pointcloud, &p.prev_pointcloud, 20, 0.0001);
                    p.prev_pose_correction = pose_correction;
                    // p.pose + pose_correction
                    motion_model_pose
                };

                // println!("scan match pose: {:?}", scan_match_pose);

                // step 3.)
                // sample points around the pose x*_t
                let translational_range = &gain.velocity.x * dt * 0.1;
                let angular_range = &gain.angular * dt * 0.1;
                // println!("trans range: {}", translational_range);
                // println!("ang range: {}", angular_range);
                let std_dev_sampling = Pose::new(Point::new(translational_range, translational_range), angular_range);

                let pose_samples: Vec<Pose> = Self::sample_distribution(&scan_match_pose, std_dev_sampling, 30);

                // step 4.)
                // compute new pose x_t drawn from the gaussian approximation of the
                // improved proposal distribution
                let (improved_pose, eta) = Self::improved_proposal(
                    &pose_samples,
                    &scan_match_pose,
                    &p.pose,
                    &p.gridmap,
                    &scan,
                    &gain,
                    dt
                );

                // step 5 & 6.)
                // update the importance weights, pose and map for particle
                p.weight = p.weight * eta;
                p.gridmap.update(&improved_pose, scan); // updating the map according to the drawn pose x_t and the observation z_t
                p.pose = improved_pose;
                p.prev_pointcloud = curr_pointcloud
            });

        // Get highest weight particle before resampling
        self.best_particle = Self::get_highest_weight_particle(&self.particles);

        println!("best pose: {:?}", self.best_particle.pose);
        println!("best weight: {:?}", self.best_particle.weight);
        println!("pose correction: {:?}", self.best_particle.prev_pose_correction);

        // step 7.)
        // compute efficient number of particles and resample based on
        // computed weights if Neff drops below threshold
        let Neff = Self::compute_neff(&self.particles);

        // TODO: do not perform resampling if robot hasn't moved since last step
        // could check if gain = 0.0
        if Neff < (*&self.particles.len() as f64) / 2.0 {
            println!("RESAMPLE!!");
            // let resampled_particles = low_variance_sampler(&self.particles);
            // self.particles = resampled_particles;
        }
    }

    pub fn compute_neff(particles: &Vec<Particle>) -> f64 {
        let squared_sum = particles
            .iter()
            .map(|p| p.weight)
            .fold(0.0, |sum, w| sum + w.powi(2));

        return 1.0 / squared_sum
    }

    fn get_highest_weight_particle(particles: &Vec<Particle>) -> Particle {
        // println!("particles: {:?}", particles);

        let particle = particles
            .iter()
            .map(|p| (p.weight, p.clone()) )
            .into_iter()
            .max_by(|x, y| match x.0.partial_cmp(&y.0) {
                Some(max) => max,
                None => panic!("Unable to get highest weight particle")
            });

        return match particle {
            Some((_, v)) => v,
            None => panic!("Could not get particle from key-value pair")
        }
    }

    #[allow(non_snake_case)]
    pub fn sample_distribution(mean: &Pose, std_dev: Pose, K: usize) -> Vec<Pose> {
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
        curr_particle_pose: &Pose,
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

        let mut mu = Pose::default(); // (0,0,0)
        let mut eta = 0.0;

        let mut poses_with_distribution: Vec<PoseWithDistribution> = vec![];

        // TODO: Run this in parallel
        sampled_poses.into_iter().for_each(|x_j: &Pose| {
            let p_x = motion_model_velocity(&x_j, prev_particle_pose, gain, dt);
            let p_z = 1.0; //likelihood_field_range_finder_model(scan, &x_j, prev_gridmap);

            mu += *x_j * p_z * p_x;
            eta += p_z * p_x;


            // to avoid re-computing, keep a cache
            poses_with_distribution.push(PoseWithDistribution { pose: *x_j, p_z, p_x });
        });

        // get final estimate of mean pose by normalizing the mean using normalization factor
        if eta > 0.0 {
            mu = mu / eta;
        }

        let mut sigma = Pose::default(); // (0,0,0)

        poses_with_distribution.into_iter().for_each(|pwd: PoseWithDistribution| {
            let (x_j, p_z, p_x) = (pwd.pose, pwd.p_z, pwd.p_x);
            sigma = sigma + (x_j - mu) * (x_j - mu) * p_z * p_x;
        });

        // get final estimate of sigma (pose variance)
        if eta > 0.0 {
            sigma = sigma / eta;
        }

        // sample final particle pose
        let improved_pose = match Self::sample_distribution(&mu, sigma.sqrt(), 1).first() {
            None => panic!("could not sample new pose!"),
            Some(p) => *p
        };

        return (improved_pose, eta)
    }
}
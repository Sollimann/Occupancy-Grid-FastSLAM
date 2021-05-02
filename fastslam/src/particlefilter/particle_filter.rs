use crate::gridmap::grid_map::GridMap;
use crate::odometry::{Pose, Twist};
use crate::sensor::laserscanner::Scan;
use crate::particlefilter::particle::Particle;
//use rayon::iter::{IntoParallelRefMutIterator, IntoParallelIterator};
use rayon::prelude::*;

#[derive(Clone, Debug)]
pub struct ParticleFilter {
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
            n_particles,
            particles: vec![],
            gridmap: init_gridmap,
            pose_estimate: init_pose
        }
    }
}

impl ParticleFilter {

    /// particles: S_t-1 - the sample set of the previous step
    /// scan: z_t - the most recent laser scan
    /// gain: u_t-1 - the most recent gain, applied in the previous step
    pub fn cycle(&mut self, scan: &Scan, gain: &Twist) {

        // This is an iterator-like chain that potentially executes in parallel
        // we iterate over all particles in the filter and do the following
        self.particles
            .par_iter_mut()
            .for_each(|p: &mut Particle| {
                p.weight = 2.0


                // step 1.)
                // initial guess of pose x'_ bade on motion model

                // step 2.)
                // scan-matching using the initial guess x'_t and the latest scan m_t
                // to compute a pose estimate x*_t

                // step 3.)
                // sample points around the pose x*_t (using nearest neighbor?)

                // step 4.)
                // compute new pose x_t drawn from the gaussian approximation of the
                // improved proposal distribution

                // step 5.)
                // update the importance weights

                // step 6.)
                // updating the map according to the drawn pose x_t and the observation z_t

                // step 7.)
                // compute efficient number of particles and resample based on
                // computed weights if Neff drops below threshold

            });

        println!("particles: {:?} ", self.particles);
    }
}
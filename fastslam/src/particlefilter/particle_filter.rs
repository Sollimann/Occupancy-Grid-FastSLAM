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

    pub fn cycle(&mut self, scan: &Scan, pose: &Pose, gain: &Twist) {

        // This is an iterator-like chain that potentially executes in parallel
        self.particles
            .par_iter_mut()
            .for_each(|p: &mut Particle| {
                p.weight = 2.0


            });

        println!("particles: {:?} ", self.particles);
    }
}
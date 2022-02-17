use rand::Rng;
use crate::particlefilter::particle::Particle;
use crate::odometry::Pose;
use crate::gridmap::grid_map::GridMap;

/// Resampling that refocuses the particle set to regions in state space with
/// high posterior probability (statistical probability that a hypothesis is true calculated in the light of relevant observations)
/// By doing so, it focuses the computational resources of the filter algorithm to regions in the
/// state space where they matter the most.
///
/// Resampling:
/// - Necessary to achieve convergence
/// - Is dangerous, since important samples might get lost ("particle depletion")
/// - Only makes sense if particle weights differ significantly (high variance)
///
/// More info:
///  - p.110 Table 4.4 in probabilistic robotics, Sebastian Thrun et al.
///
/// Transforms a particle set of M particles into another particle set of the same size
///
/// Input:
///     particles: current vector of particles
/// Returns:
///     particles: new vector of resampled particles
#[allow(non_snake_case)]
pub fn low_variance_sampler(particles: &Vec<Particle>) -> Vec<Particle> {

    assert!(particles.len() > 0);
    let mut resampled_particles: Vec<Particle> = vec![];
    println!("particle len: {}", particles.len());

    let M = particles.len() as i64;
    let M_inv = 1.0 / (M as f64);
    let r: f64 = rand::thread_rng().gen_range(0.0..M_inv);
    let mut c: f64 = particles.first().unwrap().weight;
    let mut i: usize = 0;

    for m in 0..M {
        let U = r + (m as f64) * M_inv;

        while U > c {
            i += 1;

            if i as i64 >= M {
                panic!("trying to access particle that is out of bounds")
            }

            let w_i = particles.get(i).unwrap().weight;
            c += w_i;
        }
        let mut p = particles.get(i).unwrap().clone();
        p.weight = M_inv;
        resampled_particles.push(p);
    }

    // assert_eq!(M, resampled_particles.len() as i64);

    return resampled_particles
}

#[allow(non_snake_case)]
pub fn resampler(particles: &Vec<Particle>) -> Vec<Particle> {
    let mut resampled_particles: Vec<Particle> = particles.clone();
    let M = particles.len() as i64;
    let M_inv = 1.0 / (M as f64);
    let r: f64 = rand::thread_rng().gen_range(0.0..M_inv);

    // instantiate container for cumulative sum of weights and particles poses
    let mut cum_sum: Vec<f64> = vec![];

    // initialize the sum of weights to 0
    let mut sum = 0.0;

    // iterate over all particles, accumulate weights and append poses
    for particle in particles {
        sum += particle.weight;
        cum_sum.push(sum);
    }

    // instantiate container
    let mut particle_ids: Vec<usize> = vec![];

    // iterate over all particles
    for (m, _) in particles.iter().enumerate() {
        let ref_sum = r + (m as f64) * M_inv;

        // select index of first particle for which cumulative sum exceeds reference threshold
        for (i, c) in cum_sum.iter().enumerate() {
            if c >= &ref_sum {
                let particle_id = i;
                particle_ids.push(particle_id);
                break;
            }
        }
    }

    // resample particles based on selected ids
    for (m, mut p) in resampled_particles.iter_mut().enumerate() {

        // get id of sampled particle and assign corresponding pose
        let particle_id = particle_ids[m];
        let sampled_p = particles.get(particle_id).unwrap().clone();
        *p = sampled_p;
        p.weight = M_inv;
    }

    resampled_particles
}
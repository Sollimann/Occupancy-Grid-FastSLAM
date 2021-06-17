use rand::Rng;
use crate::particlefilter::particle::Particle;

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

    let M = particles.len() as i64;
    let M_inv = 1.0 / (M as f64);
    let mut resampled_particles: Vec<Particle> = vec![];
    let r: f64 = rand::thread_rng().gen_range(0.0..M_inv);
    let mut c: f64 = particles.first().unwrap().weight;
    let mut i: usize = 1;

    for m in 1..M {
        let U = r + ((m as f64) - 1.0) * M_inv;
        while U > c {
            i += 1;

            if i as i64 >= M {
                panic!("trying to access particle that is out of bounds")
            }

            let w_i = particles.get(i).unwrap().weight;
            c += w_i;
        }
        let p = particles.get(i).unwrap();
        resampled_particles.push(p.clone());
    }

    assert_eq!(M, resampled_particles.len() as i64);

    return resampled_particles
}
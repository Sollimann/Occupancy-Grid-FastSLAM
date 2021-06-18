use fastslam::particlefilter::particle::Particle;
use fastslam::odometry::Pose;
use fastslam::gridmap::grid_map::GridMap;
use fastslam::particlefilter::resampling::low_variance_sampler;
use std::slice;

#[test]
fn test_resampling() {

    let particles: Vec<Particle> = vec![
        Particle::new(Pose::default(), 0.1, GridMap::default()),
        Particle::new(Pose::default(), 0.1, GridMap::default()),
        Particle::new(Pose::default(), 19.1, GridMap::default()),
        Particle::new(Pose::default(), 0.1, GridMap::default()),
        Particle::new(Pose::default(), 0.14, GridMap::default()),
        Particle::new(Pose::default(), 351.1, GridMap::default()),
        Particle::new(Pose::default(), 0.1, GridMap::default()),
        Particle::new(Pose::default(), 0.1, GridMap::default()),
        Particle::new(Pose::default(), 591.1, GridMap::default()),
        Particle::new(Pose::default(), 1.1, GridMap::default()),
        Particle::new(Pose::default(), 0.0001, GridMap::default()),
        Particle::new(Pose::default(), 0.000001, GridMap::default()),
    ];

    let resampled_particles = low_variance_sampler(&particles);
    let resampled_weights: Vec<f64> = resampled_particles.iter().map(|p| p.weight).collect();
    println!("resampled: {:?}", resampled_weights);
}
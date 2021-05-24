use rand_distr::{Normal, Distribution};

pub struct Noise {
    pub pose_drift: f64,
    pub std_dev_pose: f64,
    pub std_dev_laser: f64
}


pub fn gaussian(mean: f64, std_dev: f64) -> f64 {
    let normal = Normal::new(mean, std_dev).unwrap();
    normal.sample(&mut rand::thread_rng())
}



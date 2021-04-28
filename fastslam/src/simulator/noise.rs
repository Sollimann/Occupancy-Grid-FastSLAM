use rand_distr::{Normal, Distribution};

pub fn gaussian(mean: f64, std_dev: f64) -> f64 {
    let normal = Normal::new(mean, std_dev).unwrap();
    normal.sample(&mut rand::thread_rng())
}
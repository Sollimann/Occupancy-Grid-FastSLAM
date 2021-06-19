use rand_distr::{Normal, Distribution};

pub struct Noise {
    pub pose_drift: f64,
    pub std_dev_pose: f64,
    pub std_dev_laser: f64
}


pub fn gaussian(mean: f64, std_dev: f64) -> f64 {
    // create a normal distribution
    let normal = Normal::new(mean, std_dev).unwrap();

    // sample from that normal distribution
    normal.sample(&mut rand::thread_rng())
}


#[test]
fn test_gauss() {
    let g = Normal::new(0.05, 0.001).unwrap();
    let n = g.sample(&mut rand::thread_rng());
    println!("g: {:?}, n: {}", g, n);
}
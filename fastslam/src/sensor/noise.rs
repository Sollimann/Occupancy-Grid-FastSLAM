use rand_distr::{Normal, Distribution, Uniform};

pub struct Noise {
    pub std_dev_gain: f64,
    pub std_dev_laser: f64
}


pub fn gaussian(mean: f64, std_dev: f64) -> f64 {
    // create a normal distribution
    let normal = Normal::new(mean, std_dev).unwrap();

    // sample from that normal distribution
    normal.sample(&mut rand::thread_rng())
}

pub fn uniform(center: f64, interval: f64) -> f64 {
    let range = Uniform::from(-interval.abs()..interval.abs());
    let mut rng = rand::thread_rng();
    center + range.sample(&mut rng)
}


#[test]
fn test_gauss() {
    let g = Normal::new(0.05, 0.001).unwrap();
    let n = g.sample(&mut rand::thread_rng());
    println!("g: {:?}, n: {}", g, n);
}

#[test]
fn test_uniform() {
    uniform(-10.0, 1.0);
}
use fastslam::sensor::noise::gaussian;

#[test]
fn test_gaussian_noise() {
    let v = gaussian(30.0, 1.0);
    println!("v: {}", v)
}
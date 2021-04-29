use fastslam::simulator::noise::gaussian;

#[test]
fn test_gaussian_noise() {
    let v = gaussian(0.5, 0.05);
    println!("{} is from a N(2, 9) distribution", v)
}
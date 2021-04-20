use fastslam::math::utils::sigmoid;

#[test]
fn test_sigmoid() {
    let max = 6.0;
    let min = -6.0;
    let s_max= sigmoid(max);
    let s_min= sigmoid(min);

    assert!(s_max > 0.99);
    assert!(s_min < 0.01)
}
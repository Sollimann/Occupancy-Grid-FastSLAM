use fastslam::geometry::{Point, Vector};
use fastslam::pointcloud::PointCloud;
use fastslam::scanmatching::icp::best_fit_transform;
use nalgebra as na;
use approx::*;

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_identical_pointclouds() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let A = PointCloud::new(vec![p0, p1, p2, p3]);
    let B = A.clone();

    let (R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(1.0, 0.0,
                                                0.0, 1.0);

    let t_expected = na::Vector2::new(0.0, 0.0);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_shifted_pointclouds() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * 3.0, p1 * 1.0, p2 * 2.0, p3 * 1.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b.clone());

    let (R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(0.99766257, 0.06833305,
                                                -0.06833305 , 0.99766257);

    let t_expected = na::Vector2::new(1.16750548, 1.83833811);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}


#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_shifted_pointclouds_negative() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * -3.0, p1 * -2.0, p2 * -1.0, p3 * 0.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b.clone());

    let (R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(-0.983282,   -0.18208926,
                                                0.18208926, -0.983282);

    let t_expected = na::Vector2::new(-1.04328592, -2.24850907);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_pointclouds_pos_and_neg_and_switched() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * 3.0, p1 * -2.0, p2 * 1.0, p3 * 0.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b.clone());

    let (R, t) = best_fit_transform(B, A);

    let R_expected = na::Matrix2::new(0.91036648, -0.41380294,
                                      0.41380294, 0.91036648);

    let t_expected = na::Vector2::new(1.31207044, -0.09485957);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}
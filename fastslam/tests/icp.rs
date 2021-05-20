use nalgebra as na;
use na::{DMatrix, U2, Dynamic, MatrixXx2, MatrixXx3};
use fastslam::geometry::Point;
use fastslam::scanmatching::icp::{to_na_homogeneous, icp};
use fastslam::pointcloud::PointCloud;

// type Matrix2xXf64 = DMatrix<f64, U2, Dynamic>;
type Matrix2xXf64 = MatrixXx2<f64>;

#[test]
fn test_homogeneous() {
    let p0 = na::Point2::new(2.0, 2.0);
    let p1 = na::Point2::new(3.0, 3.0);
    let p2 = na::Point2::new(4.0, 4.0);
    let p3 = na::Point2::new(5.0, 5.0);
    let v = vec![p0, p1, p2, p3];

    let dm1 = Matrix2xXf64::from_vec(vec![1.0, 2.0, 3.0, 4.0]);

    let p2 = na::Point2::new(2.3, 3.5);
    let p2_homo = p2.to_homogeneous().data;
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_0() {
    let a = vec![
        Point::new(3.0, -2.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0)
    ];

    let b = vec![
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let icp = icp(&A, &B, 1, 0.0001);
}

#[test]
#[allow(non_snake_case)]
fn test_convert_to_na_homogeneous() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let A = PointCloud::new(a);
    let A_homo = to_na_homogeneous(&A);

    println!("A_homo: {}", A_homo);
}


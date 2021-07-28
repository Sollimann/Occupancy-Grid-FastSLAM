use nalgebra as na;
use na::{MatrixXx2};
use fastslam::geometry::Point;
use fastslam::scanmatching::icp::{to_na_homogeneous, icp};
use fastslam::pointcloud::PointCloud;
use fastslam::odometry::Pose;
use fastslam::math::utils::pose_relative_eq;

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
    let _ = p2.to_homogeneous().data;
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_0() {
    let a = vec![
        Point::new(1.0, 2.0),
        Point::new(2.0, 2.0),
        Point::new(3.0, -3.0),
        Point::new(-1.0, -2.0)
    ];

    let b = vec![
        Point::new(3.0, 6.0),
        Point::new(-4.0, -40.0),
        Point::new(3.0, 3.0),
        Point::new(0.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 1, 0.000001);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_1() {
    let a = vec![
        Point::new(3.0, -2.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(40.0, -10.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 100, 0.000001);
    let pose_dif_expected = Pose::new(
        Point { x: 7.729003210378913, y: 2.9526877942756187 },
        0.011227837265104052 );

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}


#[test]
#[allow(non_snake_case)]
fn test_icp_case_2() {
    let a = vec![
        Point::new(3.0, -2.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(40.0, -10.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&B, &A, 100, 0.000001);
    let pose_dif_expected = Pose::new(
        Point { x: 2.8121879293507206, y: 16.15395993803767 },
        -0.04966336822821934);
    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_3() {
    let a = vec![
        Point::new(100.0, 100.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(400.0, -100.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(-200.0, -200.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(30.0, -30.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&B, &A, 100, 0.000001);
    let pose_dif_expected = Pose::new(
        Point { x: 20.129471360126942, y: 47.34280242686634 },
        -0.18743341847319145 );

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_4() {
    let a = vec![
        Point::new(100.0, 100.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(400.0, -100.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
    ];

    let b = vec![
        Point::new(-200.0, -200.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(30.0, -30.0),

    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 100, 0.000000000000001);

    let pose_dif_expected = Pose::new(
        Point { x: -1.8648786600632477, y: -1.8806433339899442 },
        0.1396554628187165);

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_5() {
    let a = vec![
        Point::new(100.0, 100.0),
        Point::new(3.0, 2.0),
        Point::new(3.0, -2.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-10.0, 7.0),
        Point::new(-40.0, -10.0),
        Point::new(-104.0, 3.5),
        Point::new(400.0, -100.0),
        Point::new(-14.0, 37.5),
        Point::new(4.07, 1.07),
        Point::new(19.0, 36.5),
        Point::new(100.0, 200.5),
        Point::new(300.0, 400.5),
        Point::new(500.0, 600.5),
        Point::new(700.0, 800.5),
        Point::new(900.0, 1000.5),
    ];

    let b = vec![
        Point::new(-200.0, -200.0),
        Point::new(-4.0, -10.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0),
        Point::new(3.0, 3.0),
        Point::new(-10.0, 0.0),
        Point::new(-1.0, 204.0),
        Point::new(300.0, 6.0),
        Point::new(30.0, -30.0),
        Point::new(-100.0, -200.5),
        Point::new(-300.0, -400.5),
        Point::new(-500.0, -600.5),
        Point::new(-700.0, -800.5),
        Point::new(-900.0, -1000.5),
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let pose_dif = icp(&A, &B, 100, 0.00001);

    let pose_dif_expected = Pose::new(
        Point { x: -176.46623804632895, y: 24.5001641879306 },
        -0.9117716408207921);

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_icp_case_6() {
    let a = vec![
        Point::new(2.1081734290318326, -0.024841726273664107),
        Point::new(2.109651267036626, 0.09888161003103749),
        Point::new(23.36624646262446, -37.136271615135135),
        Point::new(163.31616161613, 135.13631617161361),
        Point::new(-67.31616136, -60.136161361365136),
        Point::new(1.0, 1.0),
        Point::new(59.0, 59.0),
    ];

    let b = vec![
        Point::new(3.1081734290318326, -1.024841726273664107),
        Point::new(3.109651267036626, 1.09888161003103749),
        Point::new(20.36624646262446, -40.136271615135135),
        Point::new(160.31616161613, 130.13631617161361),
        Point::new(-70.31616136, -63.136161361365136),
        Point::new(3.0, 90.0),
        Point::new(-3.0, 10.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);


    let pose_dif = icp(&A, &B, 20, 0.00001);

    let pose_dif_expected = Pose::new(
        Point { x: -10.729149492301724, y: 5.939548258818714 },
        0.040479440439105514);

    assert_eq!(pose_relative_eq(pose_dif, pose_dif_expected, 1.0e-1), true);
}
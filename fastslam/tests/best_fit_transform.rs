use fastslam::geometry::{Point, Vector};
use fastslam::pointcloud::PointCloud;
use fastslam::scanmatching::icp::{best_fit_transform, handle_improper_rotation};
use nalgebra as na;
use approx::*;

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_no_reflection_case_0() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let A = PointCloud::new(vec![p0, p1, p2, p3]);
    let B = A.clone();

    let (_, R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(1.0, 0.0,
                                                0.0, 1.0);

    let t_expected = na::Vector2::new(0.0, 0.0);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_no_reflection_case_1() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * 3.0, p1 * 1.0, p2 * 2.0, p3 * 1.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(0.99766257, 0.06833305,
                                                -0.06833305 , 0.99766257);

    let t_expected = na::Vector2::new(1.16750548, 1.83833811);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}


#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_no_reflection_case_2() {
    let a = vec![
        Point::new(1.0, 2.0),
        Point::new(2.0, 2.0),
        Point::new(3.0, 3.0),
        Point::new(-1.0, -2.0)
    ];

    let b = vec![
        Point::new(-3.0, -6.0),
        Point::new(-4.0, -4.0),
        Point::new(-3.0, -3.0),
        Point::new(0.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);
    let R_expected = na::Matrix2::new(-0.983282,   -0.18208926,
                                                0.18208926, -0.983282);

    let t_expected = na::Vector2::new(-1.04328592, -2.24850907);


    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_no_reflection_case_3() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * 3.0, p1 * -2.0, p2 * 1.0, p3 * 0.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b.clone());

    let (_, R, t) = best_fit_transform(B, A);

    let R_expected = na::Matrix2::new(0.91036648, -0.41380294,
                                      0.41380294, 0.91036648);

    let t_expected = na::Vector2::new(1.31207044, -0.09485957);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_pointcloud_reflection_case_0() {
    let a = vec![
        Point::new(1.0, -2.0),
        Point::new(2.0, 2.0),
        Point::new(3.0, 2.0),
        Point::new(-1.0, 2.0)
    ];

    let b = vec![
        Point::new(3.0, 6.0),
        Point::new(-4.0, -4.0),
        Point::new(3.0, 3.0),
        Point::new(0.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(-0.85207599, -0.52341811,
                                      0.52341811, -0.85207599);

    let t_expected = na::Vector2::new(2.08851309, 1.44780335);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_reflection_case_1() {
    let a = vec![
        Point::new(3.0, -2.0),
        Point::new(-2.0, 2.0),
        Point::new(30.0, 3.0),
        Point::new(-1.0, 204.0)
        ];

    let b = vec![
        Point::new(3.0, 6.0),
        Point::new(-4.0, -4.0),
        Point::new(3.0, 3.0),
        Point::new(0.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(-0.68895316, -0.72480586,
                                                    0.72480586, -0.68895316);

    let t_expected = na::Vector2::new(43.17585206, 31.46728233);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}


#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_no_reflection_case_4() {
    let a = vec![
        Point::new(30.0, -2.0),
        Point::new(-32.0, 2.0),
        Point::new(30.0, 332.0),
        Point::new(-1.0, 204.0)
    ];

    let b = vec![
        Point::new(3.0, 6.0),
        Point::new(-4.0, -100.0),
        Point::new(3.0, 40.0),
        Point::new(-10.0, 0.0)
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);

    let R_expected = na::Matrix2::new(0.97272514, -0.23196079,
                                      0.23196079,  0.97272514);

    let t_expected = na::Vector2::new( 22.51685114, -145.41090368);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_reflection_case_2() {
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

    let (_, R, t) = best_fit_transform(A, B);
    let R_expected = na::Matrix2::new(0.01979609, -0.99980404,
                                      0.99980404, 0.01979609);

    let t_expected = na::Vector2::new( 122.70051175,-3.24931067);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_no_reflection_case_5() {
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

    let (_, R, t) = best_fit_transform(A, B);
    let R_expected = na::Matrix2::new(-0.69839246, -0.71571501,
                                      0.71571501, -0.69839246);

    let t_expected = na::Vector2::new( 59.5285593, 9.3135397);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_reflection_case_3() {
    let a = vec![
        Point::new(1.0, 100.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(-1.0, -100.0),
    ];

    let b = vec![
        Point::new(-1.0, -100.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(1.0, 100.0),
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R1, t1) = best_fit_transform(A.clone(), B.clone());
    let R_expected = na::Matrix2::new(-1.0,  0.0,
                                      0.0, -1.0);

    let t_expected = na::Vector2::new( 0.0,0.0);

    assert_eq!(relative_eq!(R1, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t1, t_expected, epsilon = 1.0e-1), true);

    let (_, R2, t2) = best_fit_transform(B, A);

    assert_eq!(relative_eq!(R2, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t2, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_reflection_case_4() {
    let a = vec![
        Point::new(1.0, 100.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(-1.0, -100.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(500.0, -90.0),
    ];

    let b = vec![
        Point::new(-1.0, -100.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(100.0, -300.0),
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);
    let R_expected = na::Matrix2::new(0.4340702, 0.90087905,
                                                -0.90087905, 0.4340702);

    let t_expected = na::Vector2::new( -5.27942622, 12.78654903);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn test_best_fit_transform_reflection_case_5() {
    let a = vec![
        Point::new(-1900.0, 1200.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(-1.0, -100.0),
        Point::new(0.0, 100.0),
        Point::new(0.0, 0.0),
        Point::new(500.0, -90.0),
    ];

    let b = vec![
        Point::new(-1.0, -100.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(0.0, 0.0),
        Point::new(1000.0, -3000.0),
    ];

    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (_, R, t) = best_fit_transform(A, B);
    let R_expected = na::Matrix2::new(0.6243408, 0.78115208,
                                                -0.78115208, 0.6243408);

    let t_expected = na::Vector2::new( 143.80323661, -698.20176373);

    assert_eq!(relative_eq!(R, R_expected, epsilon = 1.0e-1), true);
    assert_eq!(relative_eq!(t, t_expected, epsilon = 1.0e-1), true);
}

#[test]
#[allow(non_snake_case)]
fn improper_rotation_case_1()  {
    let U = na::Matrix2::new(-0.0099995, -0.99995,
                                    -0.99995, 0.0099995);

    let Vt = na::Matrix2::new( 0.0099995, 0.99995,
                                        -0.99995,0.0099995);


    let mut R = Vt.transpose() * U.transpose();
    let S:  na::Vector2<f64> = na::Vector2::new(1.0, 0.0);

    if R.determinant() < 0.0 {
        R = handle_improper_rotation(R, U, S,Vt);
    }
}

#[test]
#[allow(non_snake_case)]
fn improper_rotation_case_2()  {
    let U = na::Matrix2::new(-0.9998181379392826, 0.01907068555730712,
                                     -0.01907068555730701, -0.9998181379392826);

    let Vt = na::Matrix2::new( 0.0007255423829884326, 0.9999997367940907,
                               0.9999997367940907, -0.0007255423829884326);


    let mut R = Vt.transpose() * U.transpose();
    let S:  na::Vector2<f64> = na::Vector2::new(1.0, 0.0);

    if R.determinant() < 0.0 {
        println!("RESULT: \n \n ");
        R = handle_improper_rotation(R, U, S, Vt);
    }
}
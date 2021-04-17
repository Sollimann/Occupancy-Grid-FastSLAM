use std::env;
use fastslam::geometry::vector::Vector;
use fastslam::math::scalar::{PI, Angle};
use std::f64;
use fastslam::math::utils::{compute_vector_abs_diff, compute_abs_diff};

#[test]
fn test_create_vector_from_angle() {
    let mut u = Vector::from_angle(0.0);
    let mut v = Vector::new(1.0,0.0);
    let (diff_x1, diff_y1) = compute_vector_abs_diff(u, v);
    assert!(diff_x1 < 1e-10);
    assert!(diff_y1 < 1e-10);

    u = Vector::from_angle(PI/4.0);
    v = Vector::new(0.7, 0.7);
    let (diff_x2, diff_y2) = compute_vector_abs_diff(u ,v);
    assert!(diff_x2 < 5e-2);
    assert!(diff_y2 < 5e-2);

    u = Vector::from_angle(PI/2.0);
    v = Vector::new(0.0, 1.0);
    let (diff_x3, diff_y3) = compute_vector_abs_diff(u ,v);
    assert!(diff_x3 < 1e-10);
    assert!(diff_y3 < 1e-10);

    u = Vector::from_angle(3.0*PI/4.0);
    v = Vector::new(-0.7, 0.7);
    let (diff_x4, diff_y4) = compute_vector_abs_diff(u ,v);
    assert!(diff_x4 < 5e-2);
    assert!(diff_y4 < 5e-2);

    u = Vector::from_angle(PI);
    v = Vector::new(-1.0, 0.0);
    let (diff_x5, diff_y5) = compute_vector_abs_diff(u ,v);
    assert!(diff_x5 < 1e-10);
    assert!(diff_y5 < 1e-10);

    u = Vector::from_angle(5.0*PI/4.0);
    v = Vector::new(-0.7, -0.7);
    let (diff_x6, diff_y6) = compute_vector_abs_diff(u ,v);
    assert!(diff_x6 < 5e-2);
    assert!(diff_y6 < 5e-2);

    u = Vector::from_angle(-PI/4.0);
    v = Vector::new(0.7, -0.7);
    let (diff_x7, diff_y7) = compute_vector_abs_diff(u ,v);
    assert!(diff_x7 < 5e-2);
    assert!(diff_y7 < 5e-2);
}

#[test]
fn test_get_angle_from_vector() {
    let mut u = Vector::new(0.7, 0.7);
    let mut desired_ang = PI/4.0;
    let mut diff_ang = compute_abs_diff(u.angle(), desired_ang);
    assert!(diff_ang < 5e-2);

    let mut u = Vector::new(-0.7, -0.7);
    let mut desired_ang = -3.0*PI/4.0;
    let mut diff_ang = compute_abs_diff(u.angle(), desired_ang);
    assert!(diff_ang < 5e-2);
}

#[test]
fn test_operator_overloading() {
    let u = Vector::new(-1.0, -1.0);
    let v = Vector::new(1.0, 1.0);

    let add = u + v;
    assert_eq!(add, Vector::new(0.0, 0.0));

    let sub1 = u - v;
    assert_eq!(sub1, Vector::new(-2.0, -2.0));

    let sub2 = v - u;
    assert_eq!(sub2, Vector::new(2.0, 2.0));

    let vec_times_scalar = u * 2.0;
    assert_eq!(vec_times_scalar, Vector::new(-2.0, -2.0));

    assert_ne!(u, v);
    assert_eq!(v, v);
}

#[test]
fn test_vector_products() {
    // dot prod
    let mut u = Vector::new(2.0,2.0);
    let mut v = Vector::new(3.0, 1.0);
    let mut dot_prod = u.dot(v);
    assert_eq!(8.0, dot_prod);

    u = Vector::new(2.0,2.10);
    v = Vector::new(-3.5, 5.3);
    dot_prod = u.dot(v);
    let mut diff = compute_abs_diff(dot_prod, 4.13);
    assert!(diff < 5e-5);

    // cross prod
    let mut cross_prod = u.cross(v);
    diff = compute_abs_diff(cross_prod, 17.95);
    assert!(diff < 1e-10);

    u = Vector::new(1.0,2.0);
    v = Vector::new(4.0,5.0);
    cross_prod = u.cross(v);
    diff = compute_abs_diff(cross_prod, -3.0);
    assert!(diff < 1e-10);
}

#[test]
fn test_vector_rotate() {
    let mut u = Vector::new(1.0, 0.0);
    let mut v = u.rotate(5.0*PI/4.0);

    let (diff_x, diff_y) = compute_vector_abs_diff(v, Vector::new(-0.707, -0.707));
    assert!(diff_x < 5e-3);
    assert!(diff_y < 5e-3);

    u = v.rotate(3.0*PI/4.0);
    let (diff_x, diff_y) = compute_vector_abs_diff(u, Vector::new(1.0, 0.0));
    assert!(diff_x < 5e-3);
    assert!(diff_y < 5e-3);

    u = u.rotate(-5.0*PI/4.0);
    let (diff_x, diff_y) = compute_vector_abs_diff(u, Vector::new(-0.707, 0.707));
    assert!(diff_x < 5e-3);
    assert!(diff_y < 5e-3);
}

#[test]
fn test_vector_length() {
    let mut u = Vector::new(3.0, 0.0);
    assert_eq!(3.0, u.length());

    u = Vector::new(-1.0, -1.0);
    assert_eq!(2.0f64.sqrt(), u.length());
}
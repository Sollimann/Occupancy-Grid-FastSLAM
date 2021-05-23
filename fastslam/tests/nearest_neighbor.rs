use fastslam::pointcloud::PointCloud;
use fastslam::scanmatching::icp::{nearest_neighbor, best_fit_transform};
use nalgebra as na;
use approx::*;
use fastslam::geometry::Point;

fn matching_f64(v1: Vec<f64>, v2: Vec<f64>) -> usize {
    v1.iter().zip(&v2).filter(|&(a, b)| relative_eq!(a, b, epsilon = 1.0e-2)).count()
}

fn matching_i64(v1: Vec<i64>, v2: Vec<i64>) -> usize {
    v1.iter().zip(&v2).filter(|&(a, b)| a==b).count()
}

#[test]
#[allow(non_snake_case)]
fn test_nn_identical_pointclouds() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let A = PointCloud::new(vec![p0, p1, p2, p3]);
    let B = A.clone();

    let (distances, indices) = nearest_neighbor(&A, &B);

    let d_matches = matching_f64(distances.clone(), vec![0.0, 0.0, 0.0, 0.0]);
    let i_matches = matching_i64(indices.clone(), vec![0, 1, 2, 3]);

    assert_eq!(d_matches, distances.len());
    assert_eq!(i_matches, indices.len());
}

#[test]
#[allow(non_snake_case)]
fn test_nn_shifted_pointclouds() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * 3.0, p1 * 1.0, p2 * 2.0, p3 * 1.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b);

    let (distances, indices) = nearest_neighbor(&A,&B);

    let d_matches = matching_f64(distances.clone(), vec![1.0, 0.0, 1.4142, 0.0]);
    let i_matches = matching_i64(indices.clone(), vec![1, 1, 1, 3]);

    assert_eq!(d_matches, distances.len());
    assert_eq!(i_matches, indices.len());

}

#[test]
#[allow(non_snake_case)]
fn test_nn_shifted_pointclouds_negative() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * -3.0, p1 * -2.0, p2 * -1.0, p3 * 0.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b.clone());

    let (distances, indices) = nearest_neighbor(&A, &B);

    let d_matches = matching_f64(distances.clone(), vec![2.23606798, 2.82842712, 4.24264069, 2.23606798]);
    let i_matches = matching_i64(indices.clone(), vec![3, 3, 3, 2]);

    assert_eq!(d_matches, distances.len());
    assert_eq!(i_matches, indices.len());

}

#[test]
#[allow(non_snake_case)]
fn test_nn_pointclouds_pos_and_neg_and_switched() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let a = vec![p0, p1, p2, p3];
    let b = vec![p0 * 3.0, p1 * -2.0, p2 * 1.0, p3 * 0.0];
    let A = PointCloud::new(a);
    let B = PointCloud::new(b.clone());

    let (distances, indices) = nearest_neighbor(&B, &A);

    let d_matches = matching_f64(distances.clone(), vec![3.0, 3.60555128, 0.0, 2.23606798]);
    let i_matches = matching_i64(indices.clone(), vec![2, 3, 2, 0]);

    assert_eq!(d_matches, distances.len());
    assert_eq!(i_matches, indices.len());
}
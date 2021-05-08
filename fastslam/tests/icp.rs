use fastslam::geometry::{Point, Vector};
use fastslam::pointcloud::PointCloud;
use fastslam::scanmatching::icp::best_fit_transform;
use nalgebra as na;

#[test]
fn test_best_fit_transform() {
    let p0 = Point::new(1.0, 2.0);
    let p1 = Point::new(2.0, 2.0);
    let p2 = Point::new(3.0, 3.0);
    let p3 = Point::new(-1.0, -2.0);
    let pca = PointCloud::new(vec![p0, p1, p2, p3]);
    //let pcb = PointCloud::new(vec![p0-p1, p2-p1, p0+p3, p0 + p1]);
    let pcb = pca.clone();

    best_fit_transform(pca, pcb);
}


#[test]
fn test_svd() {
    let mut H: na::Matrix2<f64> = na::Matrix2::new(8.75, -10.75,
                                             10.75, 14.75);

    println!("H: {} \n", H);
    H.set_row(1, &(-1.0 * H.row(1)));

    println!("-H: {} \n", H);
}
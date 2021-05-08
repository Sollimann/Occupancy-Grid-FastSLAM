use fastslam::geometry::{Point, Vector};
use fastslam::pointcloud::PointCloud;
use fastslam::scanmatching::icp::best_fit_transform;

#[test]
fn test_best_fit_transform() {
    let p0 = Point::new(-1.0, -1.0);
    let p1 = Point::new(10.0, 10.0);
    let p2 = Point::new(15.0, 4.0);
    let p3 = Point::new(1.0, 1.0);
    let pca = PointCloud::new(vec![p0, p1, p2, p3]);
    //let pcb = PointCloud::new(vec![p0-p1, p2-p1, p0+p3, p0 + p1]);
    let pcb = pca.clone();

    best_fit_transform(pca, pcb);
}
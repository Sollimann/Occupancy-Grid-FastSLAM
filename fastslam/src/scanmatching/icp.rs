use crate::pointcloud::PointCloud;
use crate::geometry::Point;
use rayon::prelude::ParallelIterator;

///
/// Calculates the least-squares best-fit transform that maps corresponding points pc_a to pc_b
/// in two-dimensions (x,y)
/// Input:
///     pc_a: pointcloud in previous step
///     pc_b: pointcloud in current step
/// Returns:
///     rot: rotation angle
///     t: translation vector
fn best_fit_transform(mut pc_a: PointCloud, mut pc_b: PointCloud) {

    // make sure dimensions are the same
    assert_eq!(pc_a.size(), pc_b.size());

    // get centroid of each point cloud
    let centroid_a = pc_a.centroid();
    let centroid_b = pc_b.centroid();

    // center point clouds
    pc_a.iter_mut().for_each(|p: &mut Point| *p = *p - centroid_a);
    pc_b.iter_mut().for_each(|p: &mut Point| *p = *p - centroid_b);
}
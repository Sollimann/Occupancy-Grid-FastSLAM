use crate::pointcloud::PointCloud;
use crate::geometry::{Point, Vector};
use rayon::prelude::ParallelIterator;
use nalgebra as na;
use crate::geometry::vector::Mat2;
use ndarray as nd;

///
/// Calculates the least-squares best-fit transform that maps corresponding points pc_a to pc_b
/// in two-dimensions (x,y)
/// Input:
///     pc_a: pointcloud in previous step
///     pc_b: pointcloud in current step
/// Returns:
///     rot: rotation angle
///     t: translation vector
pub fn best_fit_transform(mut pca: PointCloud, mut pcb: PointCloud) {

    // make sure dimensions are the same
    assert_eq!(pca.size(), pcb.size());

    // // convert my type of point to nalgebra point
    // let to_na_p2: fn(Point) -> na::Point2<f64> = |p: Point| na::Point2::new(p.x, p.y);
    // let to_na_v2: fn(Point) -> na::Vector2<f64> = |p: Point| na::Vector2::new(p.x, p.y);
    //
    //
    // // get centroid of each point cloud
    // let na_ca: na::Vector2<f64> = to_na_v2(pca.centroid());
    // let na_cb: na::Vector2<f64> = to_na_v2(pcb.centroid());
    //
    // // convert to nalgebra library and center center point clouds
    // let na_pca: Vec<na::Point2<f64>> = pca.iter().map(|p: &Point| to_na_p2(*p)).collect();
    // let na_pcb: Vec<na::Point2<f64>> = pcb.iter().map(|p: &Point| to_na_p2(*p)).collect();
    //
    // let cov = na_pca
    //     .iter()
    //     .zip(na_pcb.iter())
    //     .map(|(p, q)| (p.clone().coords-na_ca, q.clone().coords - na_cb))
    //     .map(|(p, q)| q*p.transpose())
    //     .fold(na::Matrix2::zeros(), |a, v| a + v);

    // // get centroid of each point cloud
    // let centroid_a = pca.centroid();
    // let centroid_b = pcb.centroid();
    //
    // // center point clouds
    // // pca.iter_mut().for_each(|p: &mut Point| *p = *p - centroid_a);
    // // pcb.iter_mut().for_each(|p: &mut Point| *p = *p - centroid_b);
    // let pca_centered: Vec<Vector> = pca.iter().map(|p| (*p - centroid_a).to_vec()).collect();
    // let pcb_centered: Vec<Vector> = pcb.iter().map(|p| (*p - centroid_b).to_vec()).collect();
    //
    // let cov: Mat2 = pca_centered
    //     .iter()
    //     .zip(pcb_centered.iter())
    //     .map(|(va, vb)| va.prod(*vb))
    //     .fold([[0.0, 0.0], [0.0, 0.0]], |sum, m| {
    //         [
    //             [sum[0][0] + m[0][0], sum[0][1] + m[0][1]],
    //             [sum[1][0] + m[1][0], sum[1][1] + m[1][1]]
    //         ]
    //     });

    // get centroid of each point cloud
    let nd_pca: nd::Array2<f64> = pca.to_ndarray();
    let nd_pcb: nd::Array2<f64> = pcb.to_ndarray();


    println!("pointcloud: {:?}", nd_pca);
    println!("pointcloud: {:?}", nd_pcb);
}
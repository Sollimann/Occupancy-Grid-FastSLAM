use crate::pointcloud::PointCloud;
use crate::geometry::{Point, Vector};
use rayon::prelude::ParallelIterator;
use nalgebra as na;


/// Calculates the least-squares best-fit transform that maps corresponding points pc_a to pc_b
/// in two-dimensions (x,y)
/// Input:
///     pc_a: pointcloud in previous step
///     pc_b: pointcloud in current step
/// Returns:
///     rot: rotation angle
///     t: translation vector
pub fn best_fit_transform(mut pca: PointCloud, mut pcb: PointCloud) -> (na::Matrix2<f64>, na::Vector2<f64>) {

    // make sure dimensions are the same
    assert_eq!(pca.size(), pcb.size());

    // // convert my type of point to nalgebra point
    let to_na_p2: fn(Point) -> na::Point2<f64> = |p: Point| na::Point2::new(p.x, p.y);
    let to_na_v2: fn(Point) -> na::Vector2<f64> = |p: Point| na::Vector2::new(p.x, p.y);


    // get centroid of each point cloud
    let na_ca: na::Vector2<f64> = to_na_v2(pca.centroid());
    let na_cb: na::Vector2<f64> = to_na_v2(pcb.centroid());

    // convert to nalgebra library and center center point clouds
    let na_pca: Vec<na::Point2<f64>> = pca.iter().map(|p: &Point| to_na_p2(*p)).collect();
    let na_pcb: Vec<na::Point2<f64>> = pcb.iter().map(|p: &Point| to_na_p2(*p)).collect();

    // compute cross-covariance
    let cov: na::Matrix2<f64> = na_pca
        .iter()
        .zip(na_pcb.iter())
        .map(|(p, q)| (p.clone().coords - na_ca, q.clone().coords - na_cb))
        .map(|(p, q)| q * p.transpose())
        .fold(na::Matrix2::zeros(), |sum, m| sum + m);

    // compute SVD
    let svd = na::linalg::SVD::new(cov, true, true);
    let u: na::Matrix2<f64> = svd.u.unwrap();
    let v_t: na::Matrix2<f64> = svd.v_t.unwrap();

    // Check that decomposition produces orthogonal left and right singular vectors
    assert_eq!(u.is_orthogonal(0.01), true);
    assert_eq!(v_t.is_orthogonal(0.01), true);

    // get rotation matrix
    let mut rot: na::Matrix2<f64> = v_t * u.transpose();

    // special reflection case
    if rot.determinant() < 0.0 {
        rot.set_row(1, &(-1.0 * rot.row(1)));
    }

    // compute translation offset
    let t: na::Vector2<f64> = na_cb - rot * na_ca;

    return (rot, t)
}



/// Find the nearest (Euclidean) neighbor in pca for each point in pcb
/// Input:
///     pca: pointcloud in previous step
///     pcb: pointcloud in current step
/// Returns:
///     distances: Euclidean distances of the nearest neighbor
///     indices: dst indices of the nearest neighbor
pub fn nearest_neighbor(pca: PointCloud, pcb: PointCloud) -> (Vec<f64>, Vec<i64>) {

    let mut distances: Vec<f64> = vec![];
    let mut indices: Vec<i64> = vec![];

    // iterate over all points in pointcloud pca
    pca.iter().enumerate().for_each(|(point_id, point)| {

        // initialize min_distance
        let mut min_distance: f64 = 10000.0;

        // container of nearest neighbor
        let mut min_index: i64 = -1;

        // iterate over all points in pointcloud pcb
        pcb.iter().enumerate().for_each(|(ref_id, reference)| {

            // get difference vector and euclidean disntance between points
            let dif_vector: &Vector = &reference.to_point_vec(*point);
            let distance: f64 = dif_vector.length();

            // set new minimum index if new minimum distance
            if distance < min_distance {
                min_distance = distance;
                min_index = ref_id as i64;
            }
        });

        if min_index == -1 {
            panic!("min_index has not been set!")
        }

        distances.push(min_distance);
        indices.push(min_index);
    });

    return (distances, indices)
}
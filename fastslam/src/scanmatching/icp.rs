use crate::pointcloud::PointCloud;
use crate::geometry::{Point, Vector};
use nalgebra as na;


/// Calculates the least-squares best-fit transform that maps corresponding points from A to B
/// in two-dimensions (x,y)
/// Input:
///     A: pointcloud in previous step
///     B: pointcloud in current step
/// Returns:
///     R: rotation angle
///     t: translation vector
#[allow(non_snake_case)]
pub fn best_fit_transform(A: PointCloud, B: PointCloud) -> (na::Matrix2<f64>, na::Vector2<f64>) {

    // make sure dimensions are the same
    assert_eq!(A.size(), B.size());

    // convert my type of point to nalgebra point
    let to_na_p2: fn(Point) -> na::Point2<f64> = |p: Point| na::Point2::new(p.x, p.y);
    let to_na_v2: fn(Point) -> na::Vector2<f64> = |p: Point| na::Vector2::new(p.x, p.y);

    // get centroid of each point cloud
    let centroid_A: na::Vector2<f64> = to_na_v2(A.centroid());
    let centroid_B: na::Vector2<f64> = to_na_v2(A.centroid());

    // convert to nalgebra library and center point clouds
    let A_: Vec<na::Point2<f64>> = A.iter().map(|p: &Point| to_na_p2(*p)).collect();
    let B_: Vec<na::Point2<f64>> = B.iter().map(|p: &Point| to_na_p2(*p)).collect();

    // compute cross-covariance
    let H: na::Matrix2<f64> = A_
        .iter()
        .zip(B_.iter())
        .map(|(a, b)| (a.clone().coords - centroid_A, b.clone().coords - centroid_B))
        .map(|(aa, bb)| bb * aa.transpose())
        .fold(na::Matrix2::zeros(), |sum, m| sum + m);

    println!("H: {}", H);
    // compute SVD
    let svd = na::linalg::SVD::new(H, true, true);
    let u: na::Matrix2<f64> = svd.u.unwrap();
    let v_t: na::Matrix2<f64> = svd.v_t.unwrap();

    // Check that decomposition produces orthogonal left and right singular vectors
    assert_eq!(u.is_orthogonal(0.01), true);
    assert_eq!(v_t.is_orthogonal(0.01), true);

    // get rotation matrix
    let mut R: na::Matrix2<f64> = v_t.transpose() * u.transpose();

    // special reflection case
    if R.determinant() < 0.0 {
        R.set_row(1, &(-1.0 * R.row(1)));
    }

    // compute translation offset
    let t: na::Vector2<f64> = centroid_B - R * centroid_A;

    return (R, t)
}



/// Find the nearest (Euclidean) neighbor in A for each point in B
/// Input:
///     A: pointcloud in previous step
///     B: pointcloud in current step
/// Returns:
///     distances: Euclidean distances of the nearest neighbor
///     indices: dst indices of the nearest neighbor
#[allow(non_snake_case)]
pub fn nearest_neighbor(A: PointCloud, B: PointCloud) -> (Vec<f64>, Vec<i64>) {

    let mut distances: Vec<f64> = vec![];
    let mut indices: Vec<i64> = vec![];

    // iterate over all points in pointcloud pca
    A.iter().for_each(|point| {

        // initialize min_distance
        let mut min_distance: f64 = 10000.0;

        // container of nearest neighbor
        let mut min_index: i64 = -1;

        // iterate over all points in pointcloud pcb
        B.iter().enumerate().for_each(|(ref_id, reference)| {

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
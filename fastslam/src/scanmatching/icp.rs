use crate::pointcloud::PointCloud;
use crate::geometry::{Point, Vector};
use nalgebra as na;
use std::ops::MulAssign;
use na::{U1, U2, U3, Dynamic};
use rand_distr::num_traits::abs;
use crate::odometry::Pose;
use std::borrow::Borrow;


type M3x3 = na::Matrix3<f64>;
type M2x2 = na::Matrix2<f64>;
type V2 = na::Vector2<f64>;

/// Calculates the least-squares best-fit transform that maps corresponding points from A to B
/// in two-dimensions (x,y)
/// Input:
///     A: pointcloud in previous step
///     B: pointcloud in current step
/// Returns:
///     R: rotation angle
///     t: translation vector
#[allow(non_snake_case)]
pub fn best_fit_transform(A: &PointCloud, B: &PointCloud) -> (M3x3, M2x2, V2) {

    // make sure dimensions are the same
    assert_eq!(A.size(), B.size());

    // convert my type of point to nalgebra point
    let to_na_p2: fn(Point) -> na::Point2<f64> = |p: Point| na::Point2::new(p.x, p.y);
    let to_na_v2: fn(Point) -> na::Vector2<f64> = |p: Point| na::Vector2::new(p.x, p.y);

    // get centroid of each point cloud
    let centroid_A: na::Vector2<f64> = to_na_v2(A.centroid());
    let centroid_B: na::Vector2<f64> = to_na_v2(B.centroid());

    // convert to nalgebra library and center point clouds
    let A_: Vec<na::Point2<f64>> = A.iter().map(|p: &Point| to_na_p2(*p)).collect();
    let B_: Vec<na::Point2<f64>> = B.iter().map(|p: &Point| to_na_p2(*p)).collect();

    // compute cross-covariance
    let H: na::Matrix2<f64> = A_
        .iter()
        .zip(B_.iter())
        .map(|(a, b)| (a.clone().coords - centroid_A, b.clone().coords - centroid_B))
        .map(|(aa, bb)| bb * aa.transpose())
        .fold(na::Matrix2::zeros(), |sum, m| sum + m)
        .transpose();

    // compute SVD
    // https://medium.com/machine-learning-world/linear-algebra-points-matching-with-svd-in-3d-space-2553173e8fed
    let svd = na::linalg::SVD::new(H, true, true);
    let U: na::Matrix2<f64> = svd.u.unwrap();
    let S: na::Vector2<f64> = svd.singular_values;
    let Vt: na::Matrix2<f64> = svd.v_t.unwrap();

    // Check that decomposition produces orthogonal left and right singular vectors
    assert_eq!(U.is_orthogonal(0.01), true);
    assert_eq!(Vt.is_orthogonal(0.01), true);

    // get rotation matrix
    let mut R: na::Matrix2<f64> = Vt.transpose() * U.transpose();

    // special reflection case
    if R.determinant() < 0.0 {
        R = handle_improper_rotation(R, U, S, Vt);
    }

    // compute translation offset
    let t: na::Vector2<f64> = centroid_B - R * centroid_A;


    // create transformation matrix
    let mut T: na::Matrix3<f64> = R.clone().fixed_resize(0.0);
    let t3: na::Vector3<f64> = t.fixed_resize(1.0);
    T.set_column(2, &t3);



    return (T, R, t)
}

#[allow(non_snake_case)]
pub fn handle_improper_rotation(mut R: M2x2, U: M2x2, S: V2, mut Vt: M2x2) -> M2x2 {

    let row = if S[0] < S[1] {
        0
    } else {
        1
    };


    Vt.row_mut(row).mul_assign(-1.0);
    R = Vt.transpose() * U.transpose();
    assert!(R.determinant() >= 0.0);

    return R
}


/// For each point in A (in sequence), find the nearest neighbor by index and distance
/// in the neighboring pointcloud B
/// Input:
///     A: pointcloud in previous step
///     B: pointcloud in current step
/// Returns:
///     distances: Euclidean distances of the nearest neighbor
///     indices: dst indices of the nearest neighbor
#[allow(non_snake_case)]
pub fn nearest_neighbor(A: &PointCloud, B: &PointCloud) -> (Vec<f64>, Vec<i64>) {

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


/// The Iterative Closest Point method: finds best-fit transform that maps points A on to points B
/// Input:
///     A: pointcloud in previous step
///     B: pointcloud in current step
///     init_pose: (m+1)x(m+1) homogeneous transformation
///     max_iterations: exit algorithm after max_iterations
///     tolerance: convergence criteria
/// Returns:
///     T: final homogeneous transformation that maps A on to B
///     distances: Euclidean distances (errors) of the nearest neighbor
///     i: number of iterations to converge
#[allow(non_snake_case)]
pub fn icp(A: &PointCloud, B: &PointCloud, max_iterations: usize, tolerance: f64) -> Pose {

    let mut A_trans = A.clone();

    // Homogeneous transformation matrix
    //let mut T: na::Matrix3<f64> = na::Matrix3::identity();

    let mut prev_err = 0.0;
    let mut mean_err = 0.0;

    for _ in 0..max_iterations {

        // get neighbor information
        let (distances, indices) = nearest_neighbor(&A_trans, &B);

        // Homogeneous version of A
        let A_hom = to_na_homogeneous(&A_trans);

        // Re-arrange pointcloud B according to nearest neighbors in A
        let mut B_ordered = PointCloud::empty();

        indices.into_iter().for_each(|i| {
            let point = B.get(i as usize);
            B_ordered.add(point);
        });

        println!("A_hom_trans {}", B_ordered);

        // Get best transformation matrix for current pointclouds
        let (T_t, _, _) = best_fit_transform(&A_trans, &B_ordered);

        println!("A_hom {}", A_hom);

        // Get transformed point cloud A
        let A_hom_trans: na::OMatrix<f64, Dynamic, U3> = (T_t * A_hom.transpose()).transpose();
        A_trans = from_na_homogeneous(&A_hom_trans);



        println!("A_hom_trans {}", A_hom_trans);
        println!("A_trans {}", A_trans);

        // compute mean error
        mean_err = distances.iter().sum::<f64>() / distances.len() as f64;
        if abs(prev_err - mean_err) < tolerance || mean_err > prev_err {
            break;
        }
        println!("mean err: {}", mean_err);

        // update prev error with current error
        prev_err = mean_err;
    };

    // Homogeneous transformation matrix
    let (T, _, _) = best_fit_transform(A, &A_trans);

    return to_pose(T);
}

#[allow(non_snake_case)]
fn from_na_homogeneous(A_hom: &na::OMatrix<f64, Dynamic, U3>) -> PointCloud {
    let mut A = PointCloud::empty();
    let A_trans = A_hom.clone().remove_column(2);

    A_trans.row_iter().for_each(|row| {
        println!("row {}", row);
        let r: Vec<f64> = row.iter().map(|i| i.clone() as f64).collect();
        A.add(Point::new(r[0], r[1]));
    });

    A
}

#[allow(non_snake_case)]
fn to_pose(T: M3x3) -> Pose {
    // vec of size 9
    let row: Vec<f64> = T.row_iter()
        .flat_map(|row| {
            let r: Vec<f64> = row.iter().map(|i| i.clone() as f64).collect();
            vec![r[0], r[1], r[2]]
        })
        .collect();

    // get from transformation matrix
    let T_00 = row[0];
    let T_10 = row[3];
    let T_02 = row[2];
    let T_12 = row[5];

    // derive from transformation matrix
    let dx = T_02;
    let dy = T_12;
    let dyaw = T_10.atan2(T_00); // atan2(y, x);

    Pose:: new(Point::new(dx, dy), dyaw)
}



#[allow(non_snake_case)]
pub fn to_na_homogeneous(A: &PointCloud) -> na::DMatrix<f64> {
    let to_na_p2: fn(Point) -> na::Point2<f64> = |p: Point| na::Point2::new(p.x, p.y);

    let A_homo_vec: Vec<f64> = A
        .iter()
        .map(|p| to_na_p2(*p))
        .map(|p| p.to_homogeneous())
        .flat_map(|p| {
            let d: [f64; 3] = p.data.0[0];
            vec![d[0], d[1], d[2]]
        })
        .collect();

    let A_homo_mat = na::DMatrix::from_row_slice(A.size(), 3, &A_homo_vec);
    return A_homo_mat
}
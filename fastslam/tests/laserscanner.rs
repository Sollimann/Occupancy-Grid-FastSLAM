use fastslam::sensor::laserscanner::{Measurement, Scan};
use fastslam::geometry::point::Point;
use fastslam::odometry::pose::Pose;
use fastslam::math::scalar::PI;
use fastslam::math::utils::compute_point_abs_diff;
use rayon::iter::ParallelIterator;

#[test]
fn test_measurement_to_point_straight_ahead() {
    let meas = Measurement { angle: 0.0, distance: 20.0 };
    let pose = Pose {
        position: Point { x: 0.0, y: 0.0 },
        heading: 0.0
    };
    assert_eq!(meas.to_point(&pose), Point { x: 20.0, y: 0.0 })
}

#[test]
fn test_measurement_to_point_straight_behind() {
    let meas = Measurement { angle: PI, distance: 20.0 };
    let pose = Pose {
        position: Point { x: 0.0, y: 0.0 },
        heading: 0.0
    };

    let (diff_x, diff_y) = compute_point_abs_diff(meas.to_point(&pose), Point { x: -20.0, y: 0.0 });
    assert!(diff_x < 5e-3);
    assert!(diff_y < 5e-3);
}

#[test]
fn test_measurement_to_point_straight_right() {
    let meas = Measurement { angle: -PI/2.0, distance: 20.0 };
    let pose = Pose {
        position: Point { x: 0.0, y: 0.0 },
        heading: 0.0
    };

    let (diff_x, diff_y) = compute_point_abs_diff(meas.to_point(&pose), Point { x: 0.0, y: -20.0 });
    assert!(diff_x < 5e-3);
    assert!(diff_y < 5e-3);
}

#[test]
fn test_measurement_to_point_straight_left_when_turned_180() {
    let meas = Measurement { angle: -PI/2.0, distance: 20.0 };
    let pose = Pose {
        position: Point { x: 0.0, y: 0.0 },
        heading: -PI
    };

    let (diff_x, diff_y) = compute_point_abs_diff(meas.to_point(&pose), Point { x: 0.0, y: 20.0 });
    assert!(diff_x < 5e-3);
    assert!(diff_y < 5e-3);
}

#[test]
fn test_scan_to_pointcloud() {
    let mut scan = Scan::empty();
    scan.add(Measurement { angle: 0.0, distance: 20.0 });
    scan.add(Measurement { angle: PI, distance: 20.0 });

    let pose = Pose {
        position: Point { x: -20.0, y: 0.0 },
        heading: -PI/2.0
    };
    let pc = scan.to_pointcloud(&pose);

    pc.iter().any(|&p| {
        let (diff_x, diff_y) = compute_point_abs_diff(p, Point{ x: 0.0, y: -20.0 });
        let left = diff_x < 5e-3;
        let right = diff_y < 5e-3;
        left && right
    });

    pc.iter().any(|&p| {
        let (diff_x, diff_y) = compute_point_abs_diff(p, Point{ x: 0.0, y: 20.0 });
        let left = diff_x < 5e-3;
        let right = diff_y < 5e-3;
        left && right
    });
}
use fastslam::particlefilter::probabilistic_models::{likelihood_field_range_finder_model, motion_model_velocity};
use fastslam::sensor::laserscanner::{Scan, Measurement};
use fastslam::math::scalar::PI;
use fastslam::odometry::{Pose, Twist};
use fastslam::gridmap::grid_map::GridMap;
use fastslam::geometry::{Point, Vector};

#[test]
fn test_likelihood_range_finder_empty_map_zero_prob() {
    let mut grid = GridMap::new(100, 1.0);

    let pose = Pose {
        position: Point { x: 29.0, y: 29.0 },
        heading: 0.0
    };

    let distances = [20.0; 360].to_vec();

    let measurements = distances
        .iter()
        .enumerate()
        .map(|(i, val)| Measurement{ angle: (i as f64) * PI/180.0, distance: *val })
        .collect();

    let mut scan = Scan { measurements };
    let old_grid_map = grid.clone();
    grid.update(&pose, &mut scan);

    let prob = likelihood_field_range_finder_model(&scan, &pose, &old_grid_map);

    // probability of detecting an empty world given a previous map m_t-1 should be 0.0
    assert_eq!(prob, 0.0)
}

#[test]
fn test_likelihood_range_finder_same_scan_twice() {
    let mut grid = GridMap::new(100, 1.0);

    let mut pose = Pose {
        position: Point { x: 29.0, y: 29.0 },
        heading: 0.0
    };

    let mut meas = vec![
        Measurement { angle: 0.0, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.1, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.2, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.3, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.4, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.5, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.6, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.7, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.8, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.9, distance: 10.0 },
        Measurement { angle: -PI/2.0, distance: 10.0 },
    ];

    let mut scan = Scan { measurements: meas,  };
    grid.update(&pose, &mut scan);
    let old_grid_map = grid.clone();

    pose = Pose {
        position: Point { x: 29.0, y: 29.0 },
        heading: 0.0
    };

    meas = vec![
        Measurement { angle: 0.0, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.1, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.2, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.3, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.4, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.5, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.6, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.7, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.8, distance: 10.0 },
        Measurement { angle: -PI/2.0 * 0.9, distance: 10.0 },
        Measurement { angle: -PI/2.0, distance: 10.0 },
    ];

    scan = Scan { measurements: meas};
    grid.update(&pose, &mut scan);

    let prob = likelihood_field_range_finder_model(&scan, &pose, &old_grid_map);

    // If the robot does not move from t to t+1, the probability of measuring the same map twice
    // should be really high
    assert!(prob > 1000.0)
}

#[test]
fn test_likelihood_range_finder_unlikely_second_scan() {
    let mut grid = GridMap::new(100, 1.0);

    let mut pose = Pose {
        position: Point { x: 29.0, y: 0.0 },
        heading: 0.0
    };

    let mut meas = vec![
        Measurement { angle: 0.1, distance: 10.0 },
        Measurement { angle: 0.2, distance: 10.0 },
        Measurement { angle: 0.3, distance: 10.0 },
        Measurement { angle: 0.4, distance: 10.0 },
        Measurement { angle: 0.5, distance: 10.0 },
        Measurement { angle: 0.0, distance: 10.0 },
        Measurement { angle: -0.1, distance: 10.0 },
        Measurement { angle: -0.2, distance: 10.0 },
        Measurement { angle: -0.3, distance: 10.0 },
        Measurement { angle: -0.4, distance: 10.0 },
        Measurement { angle: -0.5, distance: 10.0 },
    ];

    let mut scan = Scan { measurements: meas,  };
    grid.update(&pose, &mut scan);
    let old_grid_map = grid.clone();

    pose = Pose {
        position: Point { x: 30.0, y: 0.0 },
        heading: 0.0
    };

    let mut meas = vec![
        Measurement { angle: 0.1, distance: 11.0 },
        Measurement { angle: 0.2, distance: 11.0 },
        Measurement { angle: 0.3, distance: 11.0 },
        Measurement { angle: 0.4, distance: 11.0 },
        Measurement { angle: 0.5, distance: 11.0 },
        Measurement { angle: 0.0, distance: 11.0 },
        Measurement { angle: -0.1, distance: 11.0 },
        Measurement { angle: -0.2, distance: 11.0 },
        Measurement { angle: -0.3, distance: 11.0 },
        Measurement { angle: -0.4, distance: 11.0 },
        Measurement { angle: -0.5, distance: 11.0 },
    ];

    scan = Scan { measurements: meas};
    grid.update(&pose, &mut scan);

    let prob = likelihood_field_range_finder_model(&scan, &pose, &old_grid_map);

    // if you measure a wall a some distance and drive forward in that direction, then in t+1 the probability of
    // measuring a wall further away in the same direction is very unlikely and should produce
    // a low probability distribution
    assert!(prob > 0.0 && prob < 0.00001)
}

#[test]
fn test_likelihood_range_finder_likely_second_scan() {
    let mut grid = GridMap::new(100, 1.0);

    let mut pose = Pose {
        position: Point { x: 29.0, y: 0.0 },
        heading: 0.0
    };

    let mut meas = vec![
        Measurement { angle: 0.1, distance: 10.0 },
        Measurement { angle: 0.2, distance: 10.0 },
        Measurement { angle: 0.3, distance: 10.0 },
        Measurement { angle: 0.4, distance: 10.0 },
        Measurement { angle: 0.5, distance: 10.0 },
        Measurement { angle: 0.0, distance: 10.0 },
        Measurement { angle: -0.1, distance: 10.0 },
        Measurement { angle: -0.2, distance: 10.0 },
        Measurement { angle: -0.3, distance: 10.0 },
        Measurement { angle: -0.4, distance: 10.0 },
        Measurement { angle: -0.5, distance: 10.0 },
    ];

    let mut scan = Scan { measurements: meas,  };
    grid.update(&pose, &mut scan);
    let old_grid_map = grid.clone();

    pose = Pose {
        position: Point { x: 30.0, y: 0.0 },
        heading: 0.0
    };

    let mut meas = vec![
        Measurement { angle: 0.1, distance: 9.0 },
        Measurement { angle: 0.2, distance: 9.0 },
        Measurement { angle: 0.3, distance: 9.0 },
        Measurement { angle: 0.4, distance: 9.0 },
        Measurement { angle: 0.5, distance: 9.0 },
        Measurement { angle: 0.0, distance: 9.0 },
        Measurement { angle: -0.1, distance: 9.0 },
        Measurement { angle: -0.2, distance: 9.0 },
        Measurement { angle: -0.3, distance: 9.0 },
        Measurement { angle: -0.4, distance: 9.0 },
        Measurement { angle: -0.5, distance: 9.0 },
    ];

    scan = Scan { measurements: meas};
    grid.update(&pose, &mut scan);

    let prob = likelihood_field_range_finder_model(&scan, &pose, &old_grid_map);

    // if you measure a wall a some distance and drive forward in that direction, then in t+1 the probability of
    // measuring a wall closer than in the last step in the same direction is very likely and should produce
    // a low probability distribution
    assert!(prob > 1000.0)
}

#[test]
fn test_motion_model_velocity() {

    let prev_pose = Pose::new(Point::new(0.0, 0.0), 0.0);
    let prev_gain = Twist::new(Vector::new(1.0, 0.0), 0.0);
    let curr_pose = Pose::new(Point::new(1.0001, 0.001), 0.0001);

    let prob_sample_overcompensated = motion_model_velocity(
        &curr_pose,
        &prev_pose,
        &prev_gain,
        0.8);

    let prob_sample_accurate = motion_model_velocity(
        &curr_pose,
        &prev_pose,
        &prev_gain,
        0.98);

    let prob_sample_undercompensated = motion_model_velocity(
        &curr_pose,
        &prev_pose,
        &prev_gain,
        1.2);

    assert!(prob_sample_accurate > prob_sample_undercompensated);
    assert!(prob_sample_undercompensated > prob_sample_overcompensated);
}
use fastslam::gridmap::grid_map::{CellState, GridMap};
use fastslam::odometry::pose::Pose;
use fastslam::geometry::point::Point;
use fastslam::math::scalar::{Scalar, PI};
use fastslam::sensor::laserscanner::{Scan, Measurement};


#[test]
fn test_initialize_grid() {
    let grid = GridMap::default();
    let state = grid.cell_state(30, 30).unwrap();
    assert_eq!(state, &CellState::Void);
}

#[test]
fn test_updating_grid() {

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
    grid.update(&pose, &mut scan);

    let mut state = grid.cell_state(99, 79).unwrap();
    assert_eq!(state, &CellState::Occupied(1));

    state = grid.cell_state(99, 79).unwrap();
    assert_eq!(state, &CellState::Occupied(1));

    state = grid.cell_state(79, 99).unwrap();
    assert_eq!(state, &CellState::Occupied(1));

    state = grid.cell_state(97, 79).unwrap();
    assert_eq!(state, &CellState::Freespace);

    state = grid.cell_state(90, 79).unwrap();
    assert_eq!(state, &CellState::Freespace);

    state = grid.cell_state(80, 80).unwrap();
    assert_eq!(state, &CellState::Freespace);
}
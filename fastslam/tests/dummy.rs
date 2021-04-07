use std::env;
use fastslam::occupancy_grid::OccupancyGrid;

#[test]
fn it_works() {
    assert_eq!(2 + 2, 4);
}

#[test]
fn show_image() {
    let occ_grid = OccupancyGrid::load();
    // println!("grid: {:?}", occ_grid);
}

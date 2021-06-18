use std::fmt;
use crate::geometry::point::Point;
use crate::math::scalar::Scalar;
use crate::odometry::pose::Pose;
use crate::sensor::laserscanner::Scan;
use crate::geometry::vector::Vector;
use line_drawing::Bresenham;

#[derive(Debug, Copy, Clone, PartialEq)]
pub enum CellState {
    Occupied(u32),
    Freespace,
    Void,
}

impl Default for CellState {
    fn default() -> CellState {
        CellState::Void
    }
}

#[derive(Clone)]
pub struct GridMap {
    pub map_size: usize, // assuming quadratic map, with (0,0) in the middle
    pub cells: Vec<Vec<CellState>>,
    cell_size: Scalar, // in meter
}

/// decide how a Point should be displayed when formatting and printing
impl fmt::Display for GridMap {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "grid map size: x_dir: {}, y_dir: {}", self.map_size, self.map_size)
    }
}

impl fmt::Debug for GridMap {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "grid map size: x_dir: {}, y_dir: {}", self.map_size, self.map_size)
    }
}

impl Default for GridMap {
    fn default() -> GridMap {
        const SIZE: usize = 111;
        const CELL_SIZE: Scalar = 0.25;
        GridMap {
            map_size: SIZE,
            cells: vec![vec![CellState::Void; SIZE]; SIZE],
            cell_size: CELL_SIZE,
        }
    }
}

impl GridMap {
    pub fn new(map_size: usize, cell_size: Scalar) -> GridMap {
        GridMap {
            map_size,
            cells: vec![vec![CellState::Void; map_size]; map_size],
            cell_size,
        }
    }

    pub fn clear(&mut self) {
        self.cells = vec![vec![CellState::Void; self.map_size]; self.map_size]
    }

    pub fn get_all_occupied_cells(&self) -> Vec<Point> {
        let mut occupied: Vec<Point> = vec![];
        for (x, rows) in self.cells.iter().enumerate() {
            for (y, _) in rows.iter().enumerate() {
                match self.cell_state(x, y) {
                    None => {}
                    Some(cellstate) => {
                        if (cellstate != &CellState::Freespace) && (cellstate != &CellState::Void) {
                            occupied.push(Point::new(x as f64, y as f64));
                        }
                    }
                }
            }
        }
        return occupied
    }

    pub fn update(&mut self, pose: &Pose, scan: &Scan) {
        use self::CellState::*;
        for &m in scan.iter() {
            let p = m.to_point(pose);

            // register occupied space
            match self.world_to_map(p) {
                Some((x,y)) => {
                    let cell: &mut CellState = &mut self.cells[x][y];
                    *cell = match *cell {
                        Occupied(count) => Occupied(count + 1),
                        Freespace => Occupied(1),
                        Void => Occupied(1),
                    };
                }
                _ => {}
            }

            // register freespace
            match self.world_to_map(pose.position) {
                None => panic!("could not convert point!"),
                Some(start) => {
                    match self.world_to_map(pose.position + Vector::from_angle(pose.heading + m.angle) * m.distance) {
                        None => panic!("could not convert point!"),
                        Some(end) => {
                            let freespace = Bresenham::new(
                                (start.0 as i64, start.1 as i64),
                                (end.0 as i64, end.1 as i64))
                                .map(|(x, y)| (x as usize, y as usize))
                                .collect::<Vec<_>>();

                            for (x,y) in freespace {
                                let cell: &mut CellState = &mut self.cells[x][y];
                                *cell = match *cell {
                                    Void => Freespace,
                                    o => o,
                                };
                            }
                        }
                    }
                }
            }
        }
    }

    fn index_from_dist(&self, dist: Scalar) -> Option<usize> {
        let map_offset = (self.map_size as Scalar) / 2.0;
        let c = (dist / self.cell_size + map_offset) as i32;

        // if cell is outside grid, return None
        if c < 0 || c > self.map_size as i32 {
            None
        } else {
            Some(c as usize)
        }
    }
    /// convert from continous world coordinates to map coordinates
    pub fn world_to_map(&self, point: Point) -> Option<(usize, usize)> {
        self.index_from_dist(point.x)
            .and_then(|x| self.index_from_dist(point.y).map(|y| (x, y)))
    }

    pub fn cell_state(&self, x: usize, y: usize) -> Option<&CellState> {
        self.cells.get(x).and_then(|north| north.get(y))
    }
}

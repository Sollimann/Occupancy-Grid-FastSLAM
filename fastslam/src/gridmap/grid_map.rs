use std::fmt;
use crate::geometry::point::Point;
use crate::math::scalar::Scalar;
use crate::odometry::pose::Pose;
use crate::sensor::laserscanner::Scan;
use crate::geometry::vector::Vector;

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

pub struct GridMap {
    pub map_size: usize, // assuming quadratic map, with (0,0) in the middle
    pub cells: Vec<Vec<CellState>>,
    cell_size: Scalar, // in meter
}

/// decide how a Point should be displayed when formatting and printing
impl fmt::Display for GridMap {
    fn fmt(&self, f: &mut fmt::Formatter) -> fmt::Result {
        write!(f, "grid: {:?}", self.cells)
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

    pub fn update(&mut self, pose: &Pose, scan: &mut Scan) {
        use self::CellState::*;
        for &m in scan.iter() {
            let p = m.to_point(pose);

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

            // Find free space
            // TODO: replace by Bresenham line drawing alg
            let num = 100;
            for i in 0..num {
                let alpha = Scalar::from(i) / Scalar::from(num);
                let p = pose.position + Vector::from_angle(pose.heading + m.angle) * alpha * m.distance;

                match self.world_to_map(p) {
                    Some((x, y)) => {
                        let cell: &mut CellState = &mut self.cells[x][y];
                        *cell = match *cell {
                            Void => Freespace,
                            o => o,
                        };
                    }
                    _ => {}
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
    fn world_to_map(&self, point: Point) -> Option<(usize, usize)> {
        self.index_from_dist(point.x)
            .and_then(|x| self.index_from_dist(point.y).map(|y| (x, y)))
    }

    pub fn cell_state(&self, x: usize, y: usize) -> Option<&CellState> {
        self.cells.get(x).and_then(|north| north.get(y))
    }
}

use crate::math::scalar::Scalar;
use rayon::iter::{IntoParallelRefIterator, IntoParallelRefMutIterator, ParallelIterator};
use rayon::slice::{Iter, IterMut};
use crate::geometry::point::Point;
use crate::odometry::pose::Pose;

#[derive(Debug, Copy, Clone)]
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
    map_size: usize, // assuming quadratic map, with (0,0) in the middle
    cells: Vec<Vec<CellState>>,
    cell_size: Scalar // in meter
}

impl Default for GridMap {
    fn default() -> GridMap {
        const SIZE: usize = 111;
        const CELL_SIZE: Scalar = 0.25;
        GridMap {
            map_size: SIZE,
            cells: vec![vec![CellState::Void; SIZE]; SIZE],
            cell_size: CELL_SIZE
        }
    }
}

impl GridMap {
    pub fn clear(&mut self) {
        self.cells = vec![vec![CellState::Void; self.map_size]; self.map_size]
    }

    fn index_from_dist(&self, dist: Scalar) -> Option<usize> {
        let map_offset = (self.map_size as Scalar) / 2.0 ;
        let c = (dist / self.cell_size + map_offset) as i32;

        // if cell is outside grid, return None
        if c < 0 || c > self.map_size as i32 {
            None
        } else {
            Some(c as usize)
        }
    }

    /// convert from continous world coordinates to map coordinates
    fn world_to_map(&self, pose: Pose) -> Option<(usize, usize)> {
        self.index_from_dist(pose.position.x)
            .and_then(|x| self.index_from_dist(pose.position.y).map(|y| (x,y)))
    }

    pub fn cell_state(&self, x: usize, y: usize) -> Option<&CellState> {
        self.cells.get(x).and_then(|north| north.get(y))
    }
}

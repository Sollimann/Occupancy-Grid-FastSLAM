use crate::geometry;
use crate::gridmap::grid_map::{GridMap, CellState};
use opengl_graphics::GlGraphics;
use graphics::math::Matrix2d;
use graphics;
use graphics::{DrawState, Rectangle};
use crate::gridmap::grid_map::CellState::{Occupied, Freespace};

pub struct RenderConfig {
    pub scale: f64
}

impl RenderConfig {
    pub fn pixel_coords(&self, p: geometry::Point) -> (f64, f64) {
        // might want to revisit the signs
        //(self.scale * (p.x as f64), -self.scale * (p.y as f64))
        (self.scale * (p.x as f64), self.scale * (p.y as f64))
    }
}

const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

pub trait Draw {
    fn draw(&self, config: RenderConfig, transform: Matrix2d, gl: &mut GlGraphics);
}

impl Draw for geometry::Line {
    fn draw(&self, config: RenderConfig, transform: [[f64; 3]; 2], gl: &mut GlGraphics) {
        let line = graphics::Line::new(WHITE, 1.0);

        let (x1, y1) = config.pixel_coords(self.start);
        let (x2, y2) = config.pixel_coords(self.end);
        let coords: [f64; 4] = [x1, y1, x2, y2];

        line.draw(coords, &DrawState::default(), transform, gl);
    }
}
impl Draw for GridMap {
    fn draw(&self, config: RenderConfig, transform: [[f64; 3]; 2], gl: &mut GlGraphics) {
        let size = self.map_size;
        let cell_size = 0.1 * config.scale;

        // draw background
        let rect_bg = graphics::Rectangle::new(graphics::color::hex("333333"));
        let width = cell_size * (size as f64);
        rect_bg.draw(
            [0.0, -width, width, width],
            &DrawState::default(),
            transform,
            gl
        );

        // draw cells
        let mut draw_cell = |rect: Rectangle, r, c| {
            let x = (c as f64) * cell_size;
            // let y = -(r as f64) * cell_size;
            let y = (r as f64) * cell_size; // this might be wrong sgn

            rect.draw(
                [x, y, cell_size, cell_size],
                &DrawState::default(),
                transform,
                gl
            );
        };

        let rect_occupied = Rectangle::new(WHITE);
        let rect_freespace = Rectangle::new(graphics::color::hex("525f49"));
        for x in 0..size {
            for y in 0..size {
                match self.cell_state(x,y) {
                    Some(&Occupied(_)) => draw_cell(rect_occupied, x, y),
                    Some(&Freespace) => draw_cell(rect_freespace, x, y),
                    _ => {}
                }
            }
        }
    }
}

use crate::geometry;
use crate::simulator;
use crate::gridmap::grid_map::{GridMap, CellState};
use opengl_graphics::GlGraphics;
use graphics::math::Matrix2d;
use graphics;
use graphics::{DrawState, Rectangle, Transformed};
use crate::gridmap::grid_map::CellState::{Occupied, Freespace};
use crate::pointcloud::PointCloud;
use rayon::iter::ParallelIterator;
use crate::particlefilter::particle;
use crate::particlefilter::particle::Particle;

pub struct RenderConfig {
    pub scale: f64
}

impl RenderConfig {
    pub fn pixel_coords(&self, p: geometry::Point) -> (f64, f64) {
        // might want to revisit the signs
        (-self.scale * (p.y as f64), -self.scale * (p.x as f64))
        //(self.scale * (p.x as f64), self.scale * (p.y as f64))
    }
}

const WHITE: [f32; 4] = [1.0, 1.0, 1.0, 1.0];

pub trait Draw {
    /// config: For mapping point to pixel
    /// transform: Transformation matrix
    /// gl: The graphics implementation used to actually draw something on screen
    fn draw(&self, config: &RenderConfig, transform: Matrix2d, gl: &mut GlGraphics);
}

impl Draw for GridMap {
    fn draw(&self, config: &RenderConfig, transform: Matrix2d, gl: &mut GlGraphics) {
        let size = self.map_size;
        let cell_size = 0.1 * config.scale;

        // draw background
        let rect_bg = graphics::Rectangle::new(graphics::color::hex("333333"));
        let width = cell_size * (size as f64);
        let gui_x = -width;
        let gui_y = -width;
        rect_bg.draw(
            [gui_x, gui_y, width, width],
            &DrawState::default(),
            transform,
            gl
        );

        // draw cells
        let mut draw_cell = |rect: Rectangle, r, c| {
            // let x = (r as f64) * cell_size;
            // let y = -(c as f64) * cell_size; // this might be wrong sgn
            let gui_y = -(r as f64) * cell_size;
            let gui_x = -(c as f64) * cell_size; // this might be wrong sgn

            rect.draw(
                [gui_x, gui_y, cell_size, cell_size],
                &DrawState::default(),
                transform,
                gl
            );
        };

        let rect_occupied = Rectangle::new(WHITE);
        let rect_freespace = Rectangle::new(graphics::color::hex("525f49"));
        for r in 0..size {
            for c in 0..size {
                match self.cell_state(r,c) {
                    Some(&Occupied(_)) => draw_cell(rect_occupied, r, c),
                    Some(&Freespace) => draw_cell(rect_freespace, r, c),
                    _ => {}
                }
            }
        }
    }
}

impl Draw for Particle {
    fn draw(&self, config: &RenderConfig, transform: Matrix2d, gl: &mut GlGraphics) {

        // move the gridmap view to the top left corner of screen
        let transform_gridmap = transform.trans(-450.0, 0.0);
        self.gridmap.draw(config, transform_gridmap, gl);
    }
}

// for simulator
impl Draw for geometry::Line {
    fn draw(&self, config: &RenderConfig, transform: Matrix2d, gl: &mut GlGraphics) {
        let line = graphics::Line::new(WHITE, 1.0);

        let (x1, y1) = config.pixel_coords(self.start);
        let (x2, y2) = config.pixel_coords(self.end);
        let coords: [f64; 4] = [x1, y1, x2, y2];

        line.draw(coords, &DrawState::default(), transform, gl);
    }
}

impl Draw for simulator::Robot {
    fn draw(&self, config: &RenderConfig, transform: Matrix2d, gl: &mut GlGraphics) {
        let robot_color = graphics::color::hex("ffd42a");
        let robot_circ = graphics::ellipse::Ellipse {
            color: robot_color,
            border: None,
            resolution: 64
        };

        let robot_radius = config.scale;
        let pos = self.odom.pose.position;
        let (px, py) = config.pixel_coords(pos);

        // draw robot body in position
        robot_circ.draw(
            [0.0, 0.0, robot_radius, robot_radius],
            &Default::default(),
            transform.trans(px - robot_radius / 2.0, py - robot_radius / 2.0),
            gl
        );

        // draw robot heading angle
        let line = graphics::Line::new(robot_color, 1.0);
        let (hx ,hy) = config.pixel_coords(
            pos + geometry::Vector::from_angle(self.odom.pose.heading) * config.scale * 0.05
        );
        line.draw([px, py, hx, hy], &DrawState::default(), transform, gl);
    }
}

impl Draw for PointCloud {
    fn draw(&self, config: &RenderConfig, transform: Matrix2d, gl: &mut GlGraphics) {
        let point = graphics::ellipse::Ellipse {
            color: graphics::color::hex("FF0000"),
            border: None,
            resolution: 32
        };
        let point_radius = 0.15 * config.scale;
        for &p in self.iter() {
            let (px, py) = config.pixel_coords(p);

            point.draw(
                [0.0, 0.0, point_radius, point_radius],
                &Default::default(),
                transform.trans(px - point_radius / 2.0, py - point_radius / 2.0),
                gl
            )
        }
    }
}
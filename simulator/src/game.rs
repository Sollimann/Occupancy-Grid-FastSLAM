use opengl_graphics::GlGraphics;
use fastslam::render::{RenderConfig, Draw};
use fastslam::simulator::Robot;
use fastslam::sensor::laserscanner::Scan;
use fastslam::particlefilter::ParticleFilter;
use fastslam::geometry;
use piston_window::{RenderArgs, Key};
use graphics::{Transformed};
use piston::UpdateArgs;
use fastslam::simulator::Direction;
use std::borrow::BorrowMut;
use fastslam::odometry::Pose;
use fastslam::simulator::noise::gaussian;

pub struct Game {
    key_pressed: bool,
    gl: GlGraphics,
    pub render_config: RenderConfig,
    robot: Robot,
    last_scan: Scan,
    particle_filter: ParticleFilter,
    pub objects: Vec<geometry::Line>
}

const COLOR_BG: [f32; 4] = [0.17, 0.35, 0.62, 1.0];

impl Game {
    pub fn new(
        gl: GlGraphics,
        render_config: RenderConfig,
        robot: Robot,
        last_scan: Scan,
        particle_filter: ParticleFilter,
        objects: Vec<geometry::Line>
    ) -> Game {
        Game {
            key_pressed: false,
            gl,
            render_config,
            robot,
            last_scan,
            particle_filter,
            objects
        }
    }

    pub fn key_pressed(&mut self, key: Key) {

        let dir = match key {
            Key::Up => Some(Direction::Forward),
            Key::Down => Some(Direction::Backward),
            Key::Left => Some(Direction::Left),
            Key::Right => Some(Direction::Right),
            _ => None,
        };

        self.robot.move_forward(dir);

        if dir == None {
            self.key_pressed = false
        } else {
            self.key_pressed = true
        }
    }

    pub fn render(&mut self, args: &RenderArgs) {
        let width = args.window_size[0];
        let height = args.window_size[1];
        let(x, y) = (f64::from(width / 2.0), f64::from(height / 2.0));

        // clear screen
        graphics::clear(COLOR_BG, &mut self.gl);

        let render_config = &self.render_config;

        let objects = &self.objects;
        let robot = &self.robot;
        let pointcloud = &self.last_scan.to_pointcloud(&robot.odom.pose);
        let particle_filter = &self.particle_filter;

        self.gl.draw(args.viewport(), |c, gl| {
            let transform = c.transform.trans(x,y);

            // draw all static objects
            for o in objects {
                o.draw(render_config, transform, gl);
            }

            // draw robot
            robot.draw(render_config, transform, gl);

            // draw current laser/lidar measurements
            pointcloud.draw(render_config, transform, gl);

            // draw the internal state of the particle filter
            particle_filter.draw(render_config, transform, gl);
        });
    }

    pub fn update(&mut self, _: &UpdateArgs) {

        let apply_pose_noise = |p: &mut Pose, sig: f64| {
            p.position.x = gaussian(p.position.x, sig);
            p.position.y = gaussian(p.position.y, sig);
            p.heading = gaussian(p.heading, sig);
        };

        let apply_scan_noise = |scan: &mut Scan, sig: f64| {
            for &mut mut m in scan.measurements.iter_mut() {
                m.distance = gaussian(m.distance, sig);
                m.angle = gaussian(m.angle, sig);
            }
        };

        // perform a laser scan
        self.last_scan = self
            .robot
            .laser_scanner
            .scan(&self.robot.odom.pose, &self.objects);

        let mut sampled_pose = self.robot.odom.pose.clone();
        let mut sampled_scan = self.last_scan.clone();

        // apply noise
        if self.key_pressed {
            apply_scan_noise(&mut sampled_scan, 0.01);
            apply_pose_noise(&mut sampled_pose, 0.07);
        }

        // run perception algorithm / particle filter
        self.particle_filter.cycle(&sampled_scan, &sampled_pose);
    }
}

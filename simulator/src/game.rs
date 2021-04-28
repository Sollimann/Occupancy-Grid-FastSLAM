use std::env;
use std::fs;
use std::io::Read;
use opengl_graphics::GlGraphics;
use fastslam::render::{RenderConfig, Draw};
use fastslam::simulator::Robot;
use fastslam::sensor::laserscanner::Scan;
use fastslam::particlefilter::ParticleFilter;
use fastslam::geometry;
use piston_window::RenderArgs;
use graphics::{Transformed};
use piston::UpdateArgs;

pub struct Game {
    gl: GlGraphics,
    pub render_config: RenderConfig,
    robot: Robot,
    last_scan: Scan,
    particle_filter: ParticleFilter,
    pub objects: Vec<geometry::Line>
}

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
            gl,
            render_config,
            robot,
            last_scan,
            particle_filter,
            objects
        }
    }

    pub fn render(&mut self, args: &RenderArgs) {
        let width = args.window_size[0];
        let height = args.window_size[1];
        let(x, y) = (f64::from(width / 2.0), f64::from(height / 2.0));

        // clear screen
        graphics::clear(graphics::color::hex("f5f5f5"), &mut self.gl);

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

        // perform a laser scan
        self.last_scan = self
            .robot
            .laser_scanner
            .scan(&self.robot.odom.pose, &self.objects);

        // run perception algorithm / particle filter
        self.particle_filter.cycle(&self.last_scan, &self.robot.odom.pose);

        // Move the robot. TODO: Create a controller
        self.robot.odom.pose.position.x -= 0.003;
        self.robot.odom.pose.position.y -= 0.003;
        // self.robot.odom.pose.heading += 0.003;
    }
}

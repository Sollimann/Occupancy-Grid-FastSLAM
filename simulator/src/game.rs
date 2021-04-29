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
use fastslam::odometry::Pose;
use fastslam::simulator::noise::{Noise, gaussian};
use fastslam::geometry::Point;

pub struct Game {
    key_pressed: bool,
    count: u32,
    sim_gl: GlGraphics,
    robot: Robot,
    last_scan: Scan,
    particle_filter: ParticleFilter,
    noise: Noise,
    pub render_config: RenderConfig,
    pub objects: Vec<geometry::Line>
}

const COLOR_BG: [f32; 4] = [0.17, 0.35, 0.62, 1.0];

impl Game {
    pub fn new(
        sim_gl: GlGraphics,
        render_config: RenderConfig,
        robot: Robot,
        last_scan: Scan,
        particle_filter: ParticleFilter,
        objects: Vec<geometry::Line>
    ) -> Game {

        let noise = Noise {
            pose_drift: 0.0005,
            std_dev_pose: 0.05,
            std_dev_laser: 0.01
        };

        Game {
            key_pressed: false,
            count: 0,
            sim_gl,
            render_config,
            robot,
            last_scan,
            particle_filter,
            noise,
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
        graphics::clear(COLOR_BG, &mut self.sim_gl);

        let render_config = &self.render_config;

        let objects = &self.objects;
        let robot = &self.robot;
        let pointcloud = &self.last_scan.to_pointcloud(&robot.odom.pose);
        let particle_filter = &self.particle_filter;

        self.sim_gl.draw(args.viewport(), |c, gl| {
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

        // apply noise
        if self.key_pressed {
            let (sampled_pose, sampled_scan) =
                (self.robot.odom.pose.clone(),
                self.last_scan.clone());

            // run perception algorithm / particle filter
            self.particle_filter.cycle(&sampled_scan, &sampled_pose);
        }
        self.key_pressed = false;
    }

    fn apply_noise(&mut self, pose: Pose, scan: Scan) -> (Pose, Scan) {
        self.count += 1;

        let pose_drift = (self.count as f64) * self.noise.pose_drift;

        let apply_pose_noise = |p: Pose, sig: f64| {
            Pose {
                position: Point {
                    x: gaussian(p.position.x + pose_drift, sig),
                    y: gaussian(p.position.y + pose_drift, sig)
                },
                heading: gaussian(p.heading + pose_drift, sig)
            }
        };

        let apply_scan_noise = |scan: Scan, sig: f64| {
            for &mut mut m in scan.measurements.clone().iter_mut() {
                m.distance = gaussian(m.distance, sig);
                m.angle = gaussian(m.angle, sig);
            }
            scan
        };

        let scan = apply_scan_noise(scan, self.noise.std_dev_laser);
        let pose  = apply_pose_noise(pose, self.noise.std_dev_pose);

        (pose, scan)
    }
}

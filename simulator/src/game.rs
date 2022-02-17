use opengl_graphics::GlGraphics;
use fastslam::render::{RenderConfig, Draw};
use fastslam::simulator::Robot;
use fastslam::sensor::laserscanner::Scan;
use fastslam::geometry;
use piston_window::{RenderArgs, Key};
use graphics::{Transformed};
use piston::UpdateArgs;
use fastslam::simulator::Direction;
use fastslam::odometry::{Pose, Twist};
use fastslam::sensor::noise::{Noise, gaussian};
use fastslam::geometry::{Point, Vector};
use fastslam::particlefilter::particle::Particle;
use fastslam::particlefilter::particle_filter::ParticleFilter;

pub struct Game {
    key_pressed: bool,
    count: u32,
    sim_gl: GlGraphics,
    robot: Robot,
    last_scan: Scan,
    particle: Particle,
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
        objects: Vec<geometry::Line>
    ) -> Game {

        let noise = Noise {
            std_dev_gain: 0.01,
            std_dev_laser: 0.01
        };

        let particle_filter = ParticleFilter::default();
        let particle = particle_filter.best_particle.clone();

        Game {
            key_pressed: false,
            count: 0,
            sim_gl,
            render_config,
            robot,
            last_scan,
            particle,
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
        let best_particle = &self.particle_filter.best_particle;

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
            best_particle.draw(render_config, transform, gl);
        });
    }

    pub fn init(&mut self) {
        // perform a laser scan
        self.last_scan = self
            .robot
            .laser_scanner
            .scan(&self.robot.odom.pose, &self.objects);

        self.particle_filter.cycle(&self.last_scan, &Twist::new(Vector::new(0.0, 0.0), 0.0));
    }

    pub fn update(&mut self, _: &UpdateArgs) {

        // perform a laser scan
        self.last_scan = self
            .robot
            .laser_scanner
            .scan(&self.robot.odom.pose, &self.objects);

        // apply noise
        if self.key_pressed {
            let (gain_noisy, scan_noisy) = self.apply_noise(
                self.robot.latest_gain.clone(),
                self.last_scan.clone()
            );

            // run perception algorithm / particle filter
            //self.particle.cycle(&sampled_scan, &sampled_pose); // noisy
            //self.particle.cycle(&sampled_scan, &self.robot.odom.pose.clone()); // perfect

            self.particle_filter.cycle(&self.last_scan, &gain_noisy);

        }
        self.key_pressed = false;
    }

    fn apply_noise(&mut self, gain: Twist, scan: Scan) -> (Twist, Scan) {
        let apply_gain_noise = |u: Twist| {
            let alpha = [0.01, 0.01, 0.01, 0.01, 0.01, 0.01]; // these values can be tuned
            let v = u.velocity.x;
            let omega = u.angular;
            let v_hat = gaussian(v, (alpha[0] * v.powi(2) + alpha[1] * omega.powi(2)).sqrt());
            let omega_hat = gaussian(omega, (alpha[2] * v.powi(2) + alpha[3] * omega.powi(2)).sqrt());

            Twist {
                velocity: Vector {
                    x: v_hat,
                    y: 0.0 },
                angular: omega_hat
            }
        };

        let apply_scan_noise = |scan: Scan, sig: f64| {
            for &mut mut m in scan.measurements.clone().iter_mut() {
                m.distance = gaussian(m.distance, sig);
                m.angle = gaussian(m.angle, sig);
            }
            scan
        };

        // let scan_noisy = apply_scan_noise(scan, self.noise.std_dev_laser);
        let gain_noisy  = apply_gain_noise(gain);

        (gain_noisy, scan)
    }
}
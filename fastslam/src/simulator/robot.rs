use crate::odometry::{Odometry, Twist, MotionModel};
use crate::simulator::laserscanner::LaserScanner;
use crate::geometry::{Vector};

#[derive(Copy, Clone, PartialEq)]
pub enum Direction {
    Forward,
    Backward,
    Left,
    Right,
}

pub struct Robot {
    u: f64, // linear vel forward [m]
    w: f64, // angular vel [rad]
    pub odom: Odometry,
    pub latest_gain: Twist,
    pub laser_scanner: LaserScanner,
}

impl Default for Robot {
    fn default() -> Robot {
        Robot {
            u: 0.15,
            w: 0.08,
            odom: Odometry::default(),
            latest_gain: Twist::default(),
            laser_scanner: LaserScanner { num_columns: 100}
        }
    }
}

impl MotionModel for Robot {}

impl Robot {
    pub fn move_forward(&mut self, dir: Option<Direction>) {
        match dir {
            Some(d) => {
                let mut ds = 0.0;
                let mut dyaw = 0.0;

                match d {
                    Direction::Forward => ds += self.u,
                    Direction::Backward => ds -= self.u,
                    Direction::Left => dyaw += self.w,
                    Direction::Right => dyaw -= self.w
                }

                let gain = Twist { velocity: Vector { x: ds, y: 0.0 }, angular: dyaw };
                self.latest_gain = gain.clone();
                // self.odom.pose = Self::sample_motion_model_velocity(&self.odom.pose, &gain, 1.0);
                self.odom.pose = Self::drive(&self.odom.pose, &gain, 1.0);
            },
            None => (),
        }
    }
}
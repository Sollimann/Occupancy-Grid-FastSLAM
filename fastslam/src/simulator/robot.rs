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
    pub laser_scanner: LaserScanner,
}

impl Default for Robot {
    fn default() -> Robot {
        Robot {
            u: 0.05,
            w: 0.07,
            odom: Odometry::default(),
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
                self.odom.pose = Self::motion_model(self.odom.pose.clone(), gain, 1.0);
            },
            None => (),
        }
    }
}
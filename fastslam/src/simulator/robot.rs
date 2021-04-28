use std::f64::consts::PI;
use crate::odometry::Odometry;
use crate::simulator::laserscanner::LaserScanner;

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


impl Robot {

    fn wrap_heading(yaw: f64) -> f64 {
        let mut angle = yaw;

        while angle < -PI {
            angle += 2.0 * PI
        }
        while angle > PI {
            angle -= 2.0 * PI
        }

        angle
    }

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
                self.odom.pose.heading = Robot::wrap_heading(self.odom.pose.heading + dyaw);

                let c_yaw = (self.odom.pose.heading).cos();
                let s_yaw = (self.odom.pose.heading).sin();

                let dx = ds * c_yaw;
                let dy = ds * s_yaw;

                self.odom.pose.position.x += dx;
                self.odom.pose.position.y += dy;

            },
            None => (),
        }
    }
}
use crate::odometry::{Pose, Twist};
use crate::geometry::Point;
use crate::sensor::noise::gaussian;
use crate::math::scalar::PI;

pub trait MotionModel {
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

    fn sample_motion_model_velocity(pose: &Pose, gain: &Twist, dt: f64) -> Pose {
        let mut ds = gain.velocity.x * dt;
        let mut dyaw = gain.angular * dt;

        // if ds != 0.0 {
        //     println!("in sample motion ds");
        //     ds = gaussian(ds, 0.001);
        // }
        //
        // if dyaw != 0.0 {
        //     println!("in sample motion dyaw");
        //     dyaw = gaussian(dyaw, 0.001);
        // }

        let heading = Self::wrap_heading(pose.heading + dyaw);

        let c_yaw = (heading).cos();
        let s_yaw = (heading).sin();

        let dx = ds * c_yaw;
        let dy = ds * s_yaw;

        let x = pose.position.x + dx;
        let y = pose.position.y + dy;

        Pose {
            position: Point { x, y },
            heading
        }
    }
}
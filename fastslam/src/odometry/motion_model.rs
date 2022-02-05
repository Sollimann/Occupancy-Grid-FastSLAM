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

    fn sample_motion_model_velocity(pose: &Pose, gain: &Twist, dt: f64, apply_noise: bool) -> Pose {
        let mut ds = gain.velocity.x * dt;
        let mut dyaw = gain.angular * dt;

        if ds != 0.0 && apply_noise {
            ds = gaussian(ds, 0.02);
        }

        if dyaw != 0.0 && apply_noise {
            dyaw = gaussian(dyaw, 0.01);
        }

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

    fn sample_motion_model_velocity_v2(pose: &Pose, gain: &Twist, dt: f64) -> Pose {
        // motion noise params
        // alpha_1:2: angular error
        // alpha_3:4: translational error
        let alpha = [0.0015, 0.0015, 0.0015, 0.0015, 0.0, 0.0]; // these values can be tuned

        let x = pose.position.x;
        let y = pose.position.y;
        let theta = pose.heading;
        let v = gain.velocity.x;
        let omega = gain.angular;

        // if the robot hasn't moved since last time, return the same pose
        if v == 0.0 && omega == 0.0 {
            return *pose
        }


        let std_dev_v = (alpha[0] * v.powi(2) + alpha[1] * omega.powi(2)).sqrt();
        println!("std dev v: {}", std_dev_v);
        let v_hat = gaussian(v, std_dev_v);
        let omega_hat = gaussian(omega, (alpha[2] * v.powi(2) + alpha[3] * omega.powi(2)).sqrt());
        let gamma_hat = gaussian(0.0, (alpha[4] * v.powi(2) + alpha[5] * omega.powi(2)).sqrt());

        let x_prime = x - (v_hat / omega_hat)*theta.sin() + (v_hat / omega_hat) * (theta + omega_hat * dt).sin();
        let y_prime = y + (v_hat / omega_hat)*theta.cos() - (v_hat / omega_hat) * (theta + omega_hat * dt).cos();
        let theta_prime = theta + omega_hat * dt + gamma_hat * dt;

        Pose {
            position: Point { x: x_prime, y: y_prime },
            heading: theta_prime
        }
    }
}
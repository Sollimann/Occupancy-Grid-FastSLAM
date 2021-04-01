
use serde::Deserialize;
use std::fs::File;
use prost_types::Timestamp;

#[derive(Debug, Deserialize)]
struct SensorData {
    start_angle: f64,
    angular_resolution: f64,
    maximum_range: f64,
    ranges: Vec<f64>,
    pose: Pose,
    timestamp: f64
}

#[derive(Debug, Deserialize)]
struct Pose {
    x: f64,
    y: f64,
    theta: f64,
}

#[derive(Debug, Deserialize)]
struct LaserScan {
    start_angle: f64,
    angular_resolution: f64,
    maximum_range: f64,
    ranges: Vec<f64>
}

#[allow(dead_code)]
pub fn load() -> Vec<crate::sensors_proto::SensorData> {
    let file = File::open("data/csail_dataset.json").expect("failed to open data file");

    let decoded: Vec<SensorData> =
        serde_json::from_reader(&file).expect("failed to deserialize sensor data");

    decoded
        .into_iter()
        .map(|sensor_data| crate::sensors_proto::SensorData {
            timestamp: sensor_data.timestamp,
            pose: Some(crate::sensors_proto::Pose {
                x: sensor_data.pose.x,
                y: sensor_data.pose.y,
                theta: sensor_data.pose.theta,
            }),
            laser: Some(crate::sensors_proto::LaserScan {
                start_angle: sensor_data.start_angle,
                angular_resolution: sensor_data.angular_resolution,
                maximum_range: sensor_data.maximum_range,
                ranges: sensor_data.ranges
            }),
        })
        .collect()
}
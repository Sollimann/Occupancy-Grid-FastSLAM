
pub mod sensors_proto {
    tonic::include_proto!("sensors");
}

use sensors_proto::sensors_client::SensorsClient;
use sensors_proto::{Pose, LaserScan, SensorData, SensorsRequest};
use tonic::transport::Channel;
use std::error::Error;
use tonic::Request;

async fn print_sensor_data(client: &mut SensorsClient<Channel>) -> Result<(), Box<dyn Error>> {
    let freq = SensorsRequest {
        stream_frequency: 100
    };

    let mut stream = client
        .list_data(Request::new(freq))
        .await?
        .into_inner();

    while let Some(data) = stream.message().await? {
        println!("NOTE = {:?}", data);
    }

    Ok(())
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let mut client = SensorsClient::connect("http://[::1]:10000").await?;


    println!("\n*** SERVER STREAMING ***");
    print_sensor_data(&mut client).await?;

    Ok(())
}
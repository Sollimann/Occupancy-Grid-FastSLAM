mod data;

use std::pin::Pin;
use std::sync::Arc;
use tokio::stream;
use tonic::{Request, Response, Status};
use futures::Stream;
pub mod sensors_proto {
    tonic::include_proto!("sensors");
}

use sensors_proto::sensors_server::{Sensors, SensorsServer};
use sensors_proto::{Pose, LaserScan, SensorData, SensorsRequest};
use tonic::transport::Server;

#[derive(Debug)]
pub struct SensorsService {
    sensor_data: Arc<Vec<SensorData>>,
}

#[tonic::async_trait]
impl Sensors for SensorsService {

    type ListDataStream =
    Pin<Box<dyn Stream<Item = Result<SensorData, Status>> + Send + Sync + 'static>>;

    async fn list_data(
        &self,
        request: Request<SensorsRequest>,
    ) -> Result<Response<Self::ListDataStream>, Status> {
        unimplemented!("todo")
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    // let addr = "[::1]:10000".parse().unwrap();
    //
    // println!("SensorsServer listening on: {}", addr);
    //
    // let route_guide = SensorsService {
    //     sensor_data: Arc::new(data::load()),
    // };
    //
    // let svc = SensorsServer::new(route_guide);
    //
    // Server::builder().add_service(svc).serve(addr).await?;

    Ok(())
}
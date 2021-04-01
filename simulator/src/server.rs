mod data;

use std::pin::Pin;
use std::sync::Arc;
use tokio::stream;
use tokio::sync::mpsc;
use tonic::{Request, Response, Status};
use futures::Stream;
pub mod sensors_proto {
    tonic::include_proto!("sensors");
}

use sensors_proto::sensors_server::{Sensors, SensorsServer};
use sensors_proto::{Pose, LaserScan, SensorData, SensorsRequest};
use tonic::transport::Server;
use tokio::time::Duration;
use std::thread;

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
        let freq: u32 = request.into_inner().stream_frequency;
        let secs: f32 = 1.0 / (freq as f32);
        let millis: u64 = (secs * 1000.0) as u64;

        let (tx, rx) = mpsc::channel(4);
        let sensor_data = self.sensor_data.clone();

        tokio::spawn(async move {
            for data in &sensor_data[..] {
                println!("  => send {:?}", data);
                thread::sleep(Duration::from_millis(millis));
                tx.send(Ok(data.clone())).await.unwrap();
            }

            println!(" /// done sending");
        });

        Ok(Response::new(Box::pin(
            tokio_stream::wrappers::ReceiverStream::new(rx),
        )))
    }
}

#[tokio::main]
async fn main() -> Result<(), Box<dyn std::error::Error>> {
    let addr = "[::1]:10000".parse().unwrap();

    println!("SensorsServer listening on: {}", addr);

    let sensor_stream = SensorsService {
        sensor_data: Arc::new(data::load()),
    };

    let svc = SensorsServer::new(sensor_stream);

    Server::builder().add_service(svc).serve(addr).await?;

    Ok(())
}
use std::sync::Arc;
use axum::{
    Router,
    routing::get,
    response::IntoResponse,
    http::{StatusCode, header},
};
use tokio::sync::RwLock;
use prost::Message;

pub async fn run_server(
    current_feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>,
    port: u16,
) {
    let app = Router::new()
        .route("/gtfs-rt/vehicle-positions", get({
            let feed = current_feed.clone();
            move || get_vehicle_positions(feed.clone())
        }))
        .route("/health", get(health_check));
    
    let addr = format!("0.0.0.0:{}", port);
    println!("Starting HTTP server on {}", addr);
    
    let listener = tokio::net::TcpListener::bind(&addr).await.unwrap();
    axum::serve(listener, app).await.unwrap();
}

async fn get_vehicle_positions(
    feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>,
) -> impl IntoResponse {
    let feed_lock = feed.read().await;
    
    match &*feed_lock {
        Some(feed_msg) => {
            let mut buf = Vec::new();
            if feed_msg.encode(&mut buf).is_ok() {
                (
                    StatusCode::OK,
                    [(header::CONTENT_TYPE, "application/x-protobuf")],
                    buf,
                )
            } else {
                (
                    StatusCode::INTERNAL_SERVER_ERROR,
                    [(header::CONTENT_TYPE, "text/plain")],
                    b"Failed to encode feed".to_vec(),
                )
            }
        }
        None => (
            StatusCode::SERVICE_UNAVAILABLE,
            [(header::CONTENT_TYPE, "text/plain")],
            b"Feed not yet available".to_vec(),
        ),
    }
}

async fn health_check() -> impl IntoResponse {
    (StatusCode::OK, "OK")
}

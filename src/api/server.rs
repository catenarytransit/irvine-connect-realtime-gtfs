use crate::matcher::VehicleStateManager;
use axum::{
    Router,
    http::{StatusCode, header},
    response::{IntoResponse, Json},
    routing::get,
};
use prost::Message;
use std::sync::Arc;
use tokio::sync::RwLock;
use tower_http::cors::CorsLayer;

pub async fn run_server(
    current_feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>,
    trip_updates_feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>,
    vehicle_states: Arc<RwLock<VehicleStateManager>>,
    port: u16,
) {
    let app = Router::new()
        .route(
            "/gtfs-rt/vehicle-positions",
            get({
                let feed = current_feed.clone();
                move || get_feed(feed.clone())
            }),
        )
        .route(
            "/gtfs-rt/trip-updates",
            get({
                let feed = trip_updates_feed.clone();
                move || get_feed(feed.clone())
            }),
        )
        .route(
            "/debug/state",
            get({
                let states = vehicle_states.clone();
                move || get_debug_state(states.clone())
            }),
        )
        .route("/health", get(health_check))
        .layer(CorsLayer::permissive());

    let addr = format!("0.0.0.0:{}", port);
    println!("Starting HTTP server on {}", addr);

    let listener = match tokio::net::TcpListener::bind(&addr).await {
        Ok(l) => l,
        Err(e) => {
            eprintln!("Failed to bind to {}: {}", addr, e);
            return;
        }
    };
    if let Err(e) = axum::serve(listener, app).await {
        eprintln!("API server failed: {}", e);
    }
}

async fn get_debug_state(states: Arc<RwLock<VehicleStateManager>>) -> impl IntoResponse {
    let states_lock = states.read().await;
    // Collect states into a vec to easily return as JSON
    let all_states: Vec<_> = states_lock.all_states().cloned().collect();
    Json(all_states)
}

async fn get_feed(feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>) -> impl IntoResponse {
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

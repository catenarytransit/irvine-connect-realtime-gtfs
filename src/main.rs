mod gtfs;
mod realtime;
mod matcher;
mod api;

use std::sync::Arc;
use tokio::sync::RwLock;

#[tokio::main]
async fn main() {
    println!("Starting Irvine Connect GTFS-RT service...");
    
    let gtfs_data = match gtfs::loader::load_gtfs().await {
        Ok(data) => {
            println!("Loaded {} stops and {} trips", data.stops.len(), data.trips.len());
            Arc::new(data)
        }
        Err(e) => {
            eprintln!("Failed to load GTFS data: {}", e);
            return;
        }
    };
    
    let vehicle_states = Arc::new(RwLock::new(matcher::VehicleStateManager::new()));
    let current_feed = Arc::new(RwLock::new(None::<gtfs_realtime::FeedMessage>));
    
    let fetcher_gtfs = gtfs_data.clone();
    let fetcher_states = vehicle_states.clone();
    let fetcher_feed = current_feed.clone();
    
    let fetcher_handle = tokio::spawn(async move {
        realtime::fetcher::run_fetcher(fetcher_gtfs, fetcher_states, fetcher_feed).await;
    });
    
    let api_feed = current_feed.clone();
    let api_handle = tokio::spawn(async move {
        api::server::run_server(api_feed).await;
    });
    
    tokio::select! {
        _ = fetcher_handle => eprintln!("Fetcher task exited"),
        _ = api_handle => eprintln!("API server exited"),
    }
}

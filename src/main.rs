mod api;
mod gtfs;
mod matcher;
mod realtime;

use clap::Parser;
use std::sync::Arc;
use tokio::sync::RwLock;

#[derive(Parser)]
#[command(name = "irvine-connect-realtime-gtfs")]
#[command(about = "GTFS-RT service for Irvine Connect")]
struct Args {
    /// Port to run the HTTP server on
    #[arg(short, long, env = "SERVER_PORT", default_value = "8080")]
    port: u16,
}

#[tokio::main]
async fn main() {
    let args = Args::parse();

    println!("Starting Irvine Connect GTFS-RT service...");

    let gtfs_data = match gtfs::loader::load_gtfs().await {
        Ok(data) => {
            println!(
                "Loaded {} stops and {} trips",
                data.stops.len(),
                data.trips.len()
            );
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
    let port = args.port;
    let api_handle = tokio::spawn(async move {
        api::server::run_server(api_feed, port).await;
    });

    tokio::select! {
        _ = fetcher_handle => eprintln!("Fetcher task exited"),
        _ = api_handle => eprintln!("API server exited"),
    }
}

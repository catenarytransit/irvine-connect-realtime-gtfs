use crate::gtfs::GtfsData;
use crate::matcher::{VehicleStateManager, algorithm, updates};
use prost::Message;
use std::sync::Arc;
use std::time::Duration;
use tokio::sync::RwLock;

const REALTIME_URL: &str =
    "https://passio3.com/irvine/passioTransit/gtfs/realtime/vehiclePositions";
const FETCH_INTERVAL_MS: u64 = 1000;

pub async fn run_fetcher(
    gtfs: Arc<GtfsData>,
    states: Arc<RwLock<VehicleStateManager>>,
    current_feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>,
    trip_updates_feed: Arc<RwLock<Option<gtfs_realtime::FeedMessage>>>,
) {
    println!(
        "Starting realtime fetcher, polling every {}ms",
        FETCH_INTERVAL_MS
    );
    let client = reqwest::Client::new();

    let mut current_iteration = 0;

    loop {
        match fetch_and_process(&client, &gtfs, &states, &current_feed, &trip_updates_feed).await {
            Ok(count) => {
                println!("Processed {} vehicles", count);
            }
            Err(e) => {
                eprintln!("Fetch error: {}", e);
            }
        }

        current_iteration += 1;
        if current_iteration % 30 == 0 {
            let state_manager = states.read().await;
            if let Err(e) = state_manager.save("vehicle_state.json") {
                eprintln!("Failed to save vehicle state: {}", e);
            } else {
                println!("Saved vehicle state");
            }
        }

        tokio::time::sleep(Duration::from_millis(FETCH_INTERVAL_MS)).await;
    }
}

async fn fetch_and_process(
    client: &reqwest::Client,
    gtfs: &GtfsData,
    states: &RwLock<VehicleStateManager>,
    current_feed: &RwLock<Option<gtfs_realtime::FeedMessage>>,
    trip_updates_feed: &RwLock<Option<gtfs_realtime::FeedMessage>>,
) -> Result<usize, Box<dyn std::error::Error + Send + Sync>> {
    let response = client.get(REALTIME_URL).send().await?;
    let bytes = response.bytes().await?;

    let feed = gtfs_realtime::FeedMessage::decode(bytes.as_ref())?;

    let timestamp = feed.header.timestamp.unwrap_or_else(|| {
        std::time::SystemTime::now()
            .duration_since(std::time::UNIX_EPOCH)
            .unwrap()
            .as_secs()
    });

    let mut new_entities = Vec::new();
    let mut vehicle_count = 0;

    {
        let mut state_manager = states.write().await;

        // Pass 1: Update positions and stops for all vehicles
        for entity in &feed.entity {
            if let Some(vehicle) = &entity.vehicle {
                if let Some(position) = &vehicle.position {
                    let vehicle_ref = vehicle.vehicle.as_ref();
                    let label = vehicle_ref.and_then(|v| v.label.clone());
                    let source_id = vehicle_ref.and_then(|v| v.label.clone());

                    let vehicle_id = label.clone().unwrap_or_else(|| entity.id.clone());

                    let state = state_manager.get_or_create(&vehicle_id);

                    // Update static info
                    if state.label.is_none() && label.is_some() {
                        state.label = label;
                    }
                    if state.source_id.is_none() && source_id.is_some() {
                        state.source_id = source_id;
                    }

                    algorithm::update_vehicle_state(
                        state,
                        position.latitude as f64,
                        position.longitude as f64,
                        position.bearing,
                        timestamp,
                        gtfs,
                    );
                }
            }
        }

        // Pass 2: Perform global assignment
        algorithm::perform_global_assignment(&mut state_manager, gtfs);

        // Pass 3: Construct new feed entities with assigned trips
        for entity in &feed.entity {
            if let Some(vehicle) = &entity.vehicle {
                // Re-derive ID to look up state
                let vehicle_id = vehicle
                    .vehicle
                    .as_ref()
                    .and_then(|v| v.label.clone())
                    .unwrap_or_else(|| entity.id.clone());

                if let Some(state) = state_manager.get(&vehicle_id) {
                    let new_vehicle = create_enhanced_vehicle(
                        vehicle,
                        state.assigned_trip_id.as_deref(),
                        state.assigned_start_date.as_deref(),
                    );

                    new_entities.push(gtfs_realtime::FeedEntity {
                        id: entity.id.clone(),
                        is_deleted: entity.is_deleted,
                        trip_update: None,
                        vehicle: Some(new_vehicle),
                        alert: None,
                        shape: None,
                        stop: None,
                        trip_modifications: None,
                    });

                    vehicle_count += 1;
                }
            }
        }
    }

    let new_feed = gtfs_realtime::FeedMessage {
        header: gtfs_realtime::FeedHeader {
            gtfs_realtime_version: "2.0".to_string(),
            incrementality: Some(0),
            timestamp: Some(timestamp),
            feed_version: None,
        },
        entity: new_entities,
    };

    {
        let mut feed_lock = current_feed.write().await;
        *feed_lock = Some(new_feed);
    }

    // Generate Trip Updates
    {
        let state_manager = states.read().await;

        let updates_msg = updates::generate_trip_updates(&gtfs, &state_manager);
        let mut updates_lock = trip_updates_feed.write().await;
        *updates_lock = Some(updates_msg);
    }

    Ok(vehicle_count)
}

fn create_enhanced_vehicle(
    original: &gtfs_realtime::VehiclePosition,
    trip_id: Option<&str>,
    start_date: Option<&str>,
) -> gtfs_realtime::VehiclePosition {
    let trip = trip_id.map(|tid| gtfs_realtime::TripDescriptor {
        trip_id: Some(tid.to_string()),
        route_id: Some("5956".to_string()),
        direction_id: None,
        start_time: None,
        start_date: start_date.map(|s| s.to_string()),
        schedule_relationship: None,
        modified_trip: None,
    });

    let vehicle = original
        .vehicle
        .as_ref()
        .map(|v| gtfs_realtime::VehicleDescriptor {
            id: v.label.clone(),
            label: v.label.clone(),
            license_plate: v.license_plate.clone(),
            wheelchair_accessible: v.wheelchair_accessible,
        });

    gtfs_realtime::VehiclePosition {
        trip,
        vehicle,
        position: original.position.clone(),
        current_stop_sequence: original.current_stop_sequence,
        stop_id: original.stop_id.clone(),
        current_status: original.current_status,
        timestamp: original.timestamp,
        congestion_level: original.congestion_level,
        occupancy_status: original.occupancy_status,
        occupancy_percentage: original.occupancy_percentage,
        multi_carriage_details: original.multi_carriage_details.clone(),
    }
}

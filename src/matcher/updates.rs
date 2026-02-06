use crate::gtfs::GtfsData;
use crate::matcher::history::VehicleStateManager;
use gtfs_realtime::FeedMessage;
use std::time::{SystemTime, UNIX_EPOCH};

pub fn generate_trip_updates(gtfs: &GtfsData, states: &VehicleStateManager) -> FeedMessage {
    let mut feed = FeedMessage::default();

    let current_time = SystemTime::now()
        .duration_since(UNIX_EPOCH)
        .unwrap()
        .as_secs();

    let mut header = gtfs_realtime::FeedHeader::default();
    header.gtfs_realtime_version = "2.0".to_string();
    header.timestamp = Some(current_time);
    feed.header = header;

    for state in states.all_states() {
        let vehicle_id = &state.vehicle_id;
        if let Some(trip_id) = &state.assigned_trip_id {
            if let Some(trip_update) = generate_single_trip_update(gtfs, state, trip_id) {
                let mut entity = gtfs_realtime::FeedEntity::default();
                entity.id = format!("tu-{}", vehicle_id);
                entity.trip_update = Some(trip_update);
                feed.entity.push(entity);
            }
        }
    }

    feed
}

fn generate_single_trip_update(
    gtfs: &GtfsData,
    state: &crate::matcher::history::VehicleState,
    trip_id: &str,
) -> Option<gtfs_realtime::TripUpdate> {
    let trip = gtfs.get_trip_by_id(trip_id)?;
    // In GtfsData, stop_times are embedded in the trip
    let stop_times = &trip.stop_times;

    let mut trip_update = gtfs_realtime::TripUpdate::default();
    let mut trip_descriptor = gtfs_realtime::TripDescriptor::default();
    trip_descriptor.trip_id = Some(trip_id.to_string());
    trip_descriptor.route_id = Some(trip.route_id.clone());

    if let Some(start_date) = &state.assigned_start_date {
        trip_descriptor.start_date = Some(start_date.clone());
    }

    trip_update.trip = trip_descriptor;
    trip_update.vehicle = Some(gtfs_realtime::VehicleDescriptor {
        id: Some(state.vehicle_id.clone()),
        ..Default::default()
    });

    let mut current_delay_secs = 0i32;

    // Use visited_stops logical ordering to differentiate past/future
    let mut visited_idx = 0;

    let mut stop_time_updates = Vec::new();

    for st in stop_times {
        let mut stu = gtfs_realtime::trip_update::StopTimeUpdate::default();
        stu.stop_sequence = Some(st.sequence);
        stu.stop_id = Some(st.stop_id.clone());

        let mut found_visit = None;
        if visited_idx < state.visited_stops.len() {
            if state.visited_stops[visited_idx] == st.stop_id {
                if let Some((_, timestamp)) = state.stop_visit_timestamps.get(visited_idx) {
                    found_visit = Some(*timestamp);
                }
                visited_idx += 1;
            }
        }

        if let Some(visit_ts) = found_visit {
            // Past stop
            let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
            event.time = Some(visit_ts as i64);
            stu.arrival = Some(event.clone());
            stu.departure = Some(event);
            stu.schedule_relationship = Some(
                gtfs_realtime::trip_update::stop_time_update::ScheduleRelationship::Scheduled
                    as i32,
            );

            // Calculate delay
            if let Some(date_str) = &state.assigned_start_date {
                if let Some(scheduled_secs) = st.arrival_time_secs {
                    if let Ok(scheduled_ts) = get_scheduled_timestamp(date_str, scheduled_secs) {
                        let diff = (visit_ts as i64) - (scheduled_ts as i64);
                        current_delay_secs = diff as i32;
                    }
                }
            }
        } else {
            // Future stop
            let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
            event.delay = Some(current_delay_secs);

            if let Some(date_str) = &state.assigned_start_date {
                if let Some(scheduled_secs) = st.arrival_time_secs {
                    if let Ok(scheduled_ts) = get_scheduled_timestamp(date_str, scheduled_secs) {
                        event.time = Some(scheduled_ts as i64 + current_delay_secs as i64);
                    }
                }
            }

            stu.arrival = Some(event.clone());
            stu.departure = Some(event);
            stu.schedule_relationship = Some(
                gtfs_realtime::trip_update::stop_time_update::ScheduleRelationship::Scheduled
                    as i32,
            );
        }

        stop_time_updates.push(stu);
    }

    trip_update.stop_time_update = stop_time_updates;
    Some(trip_update)
}

fn get_scheduled_timestamp(date_str: &str, secs_since_midnight: u32) -> Result<u64, ()> {
    use chrono::{NaiveDate, TimeZone};
    use chrono_tz::America::Los_Angeles;

    let date = NaiveDate::parse_from_str(date_str, "%Y%m%d").map_err(|_| ())?;
    let naive_dt = date.and_hms_opt(0, 0, 0).unwrap();
    let base_dt = Los_Angeles.from_local_datetime(&naive_dt).single().unwrap();
    let scheduled_dt = base_dt + chrono::Duration::seconds(secs_since_midnight as i64);

    Ok(scheduled_dt.timestamp() as u64)
}

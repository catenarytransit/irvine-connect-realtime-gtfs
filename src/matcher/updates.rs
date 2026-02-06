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
        if let Some(trip_id) = &state.assigned_trip_id {
            if let Some(trip_update) =
                generate_single_trip_update(gtfs, state, trip_id, current_time)
            {
                let mut entity = gtfs_realtime::FeedEntity::default();
                entity.id = format!("tu-{}", state.vehicle_id);
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
    _current_time: u64,
) -> Option<gtfs_realtime::TripUpdate> {
    let trip = gtfs.get_trip_by_id(trip_id)?;
    let stop_times = &trip.stop_times;
    let date_str = state.assigned_start_date.as_ref()?;

    let mut trip_update = gtfs_realtime::TripUpdate::default();
    let mut trip_descriptor = gtfs_realtime::TripDescriptor::default();
    trip_descriptor.trip_id = Some(trip_id.to_string());
    trip_descriptor.route_id = Some(trip.route_id.clone());
    trip_descriptor.start_date = Some(date_str.clone());
    trip_update.trip = trip_descriptor;

    trip_update.vehicle = Some(gtfs_realtime::VehicleDescriptor {
        id: Some(state.vehicle_id.clone()),
        ..Default::default()
    });

    // Build a map of stop_id -> (visit_index, actual_timestamp) from the vehicle's history
    let mut stop_visits: std::collections::HashMap<&str, (usize, u64)> =
        std::collections::HashMap::new();
    for (i, (stop_id, ts)) in state.stop_visit_timestamps.iter().enumerate() {
        stop_visits.insert(stop_id.as_str(), (i, *ts));
    }

    // Match visited stops to stop_times in sequence order
    // For each stop in the trip, check if it was visited
    let mut matched_stops: Vec<Option<u64>> = vec![None; stop_times.len()];
    let mut last_visit_idx: Option<usize> = None;

    for (st_idx, st) in stop_times.iter().enumerate() {
        if let Some(&(visit_idx, actual_ts)) = stop_visits.get(st.stop_id.as_str()) {
            // Only match if this visit comes after previous matched visits (sequence order)
            let is_valid = match last_visit_idx {
                Some(prev_idx) => visit_idx > prev_idx,
                None => true,
            };
            if is_valid {
                matched_stops[st_idx] = Some(actual_ts);
                last_visit_idx = Some(visit_idx);
            }
        }
    }

    // Find the last visited stop and calculate its delay
    let mut last_visited_idx: Option<usize> = None;
    let mut propagated_delay: i32 = 0;

    for (i, matched_ts) in matched_stops.iter().enumerate().rev() {
        if matched_ts.is_some() {
            last_visited_idx = Some(i);
            break;
        }
    }

    if let Some(idx) = last_visited_idx {
        if let Some(actual_ts) = matched_stops[idx] {
            let st = &stop_times[idx];
            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    propagated_delay = (actual_ts as i64 - sched_ts as i64) as i32;
                }
            }
        }
    }

    // Build stop time updates
    let mut stop_time_updates = Vec::new();

    for (i, st) in stop_times.iter().enumerate() {
        let mut stu = gtfs_realtime::trip_update::StopTimeUpdate::default();
        stu.stop_sequence = Some(st.sequence);
        stu.stop_id = Some(st.stop_id.clone());

        if let Some(actual_ts) = matched_stops[i] {
            // Past stop: use actual arrival time
            let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
            event.time = Some(actual_ts as i64);
            stu.arrival = Some(event.clone());
            stu.departure = Some(event);
        } else {
            // Future stop: propagate delay from last visited stop
            let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
            event.delay = Some(propagated_delay);

            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            }

            stu.arrival = Some(event.clone());
            stu.departure = Some(event);
        }

        stu.schedule_relationship = Some(
            gtfs_realtime::trip_update::stop_time_update::ScheduleRelationship::Scheduled as i32,
        );
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

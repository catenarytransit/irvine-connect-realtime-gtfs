use crate::gtfs::GtfsData;
use crate::matcher::history::VehicleStateManager;
use crate::matcher::proximity::haversine_distance;
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
    current_time: u64,
) -> Option<gtfs_realtime::TripUpdate> {
    let trip = gtfs.get_trip_by_id(trip_id)?;
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

    // --- ALIGNMENT & DELAY CALCULATION ---

    // 1. Identify which stops have been visited.
    // matched_visits[i] = index in visited_stops that matches stop_times[i], or None
    let mut matched_visits: Vec<Option<usize>> = vec![None; stop_times.len()];
    let mut visit_cursor = 0;

    // Greedy matching
    for (st_idx, st) in stop_times.iter().enumerate() {
        for v_idx in visit_cursor..state.visited_stops.len() {
            if state.visited_stops[v_idx] == st.stop_id {
                matched_visits[st_idx] = Some(v_idx);
                visit_cursor = v_idx + 1;
                break;
            }
        }
    }

    // 2. Calculate Current Delay
    let mut estimated_delay = 0i32;

    // Find the index of the last visited stop in stop_times
    let last_visited_idx = matched_visits.iter().rposition(|v| v.is_some());

    if let Some(last_idx) = last_visited_idx {
        // We have visited at least one stop.
        // Check if we can interpolate to the NEXT stop (last_idx + 1)
        if last_idx + 1 < stop_times.len() {
            let prev_st = &stop_times[last_idx];
            let next_st = &stop_times[last_idx + 1];

            if let Some(pos) = state.position_history.back() {
                estimated_delay = calculate_interpolated_delay(
                    gtfs,
                    state,
                    prev_st,
                    next_st,
                    pos.lat,
                    pos.lon,
                    current_time,
                )
                .unwrap_or(0) as i32;

                // Fallback: if interpolation fails (e.g. 0), use delay at last stop
                if estimated_delay == 0 {
                    if let Some(v_idx) = matched_visits[last_idx] {
                        if let Some((_, visit_ts)) = state.stop_visit_timestamps.get(v_idx) {
                            if let Some(date_str) = &state.assigned_start_date {
                                if let Some(sched_secs) = prev_st.arrival_time_secs {
                                    if let Ok(sched_ts) =
                                        get_scheduled_timestamp(date_str, sched_secs)
                                    {
                                        estimated_delay =
                                            (*visit_ts as i64 - sched_ts as i64) as i32;
                                    }
                                }
                            }
                        }
                    }
                }
            }
        }
    } else {
        // No stops visited yet.
        if let Some(first_st) = stop_times.first() {
            if let Some(date_str) = &state.assigned_start_date {
                if let Some(sched_secs) = first_st.arrival_time_secs {
                    if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                        if current_time > sched_ts + 60 {
                            estimated_delay = (current_time as i64 - sched_ts as i64) as i32;
                        }
                    }
                }
            }
        }
    }

    // --- CONSTRUCT UPDATES ---

    let mut stop_time_updates = Vec::new();

    for (i, st) in stop_times.iter().enumerate() {
        let mut stu = gtfs_realtime::trip_update::StopTimeUpdate::default();
        stu.stop_sequence = Some(st.sequence);
        stu.stop_id = Some(st.stop_id.clone());

        if let Some(v_idx) = matched_visits[i] {
            // PAST STOP
            if let Some((_, visit_ts)) = state.stop_visit_timestamps.get(v_idx) {
                let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
                event.time = Some(*visit_ts as i64);
                stu.arrival = Some(event.clone());
                stu.departure = Some(event);
                stu.schedule_relationship = Some(
                    gtfs_realtime::trip_update::stop_time_update::ScheduleRelationship::Scheduled
                        as i32,
                );
            }
        } else {
            // FUTURE STOP
            let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
            event.delay = Some(estimated_delay);

            if let Some(date_str) = &state.assigned_start_date {
                if let Some(sched_secs) = st.arrival_time_secs {
                    if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                        event.time = Some(sched_ts as i64 + estimated_delay as i64);
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

fn calculate_interpolated_delay(
    gtfs: &GtfsData,
    state: &crate::matcher::history::VehicleState,
    prev_st: &crate::gtfs::StopTime,
    next_st: &crate::gtfs::StopTime,
    lat: f64,
    lon: f64,
    current_time: u64,
) -> Option<i64> {
    let prev_stop = gtfs.stops.get(&prev_st.stop_id)?;
    let next_stop = gtfs.stops.get(&next_st.stop_id)?;

    let prev_time = prev_st.arrival_time_secs?;
    let next_time = next_st.arrival_time_secs?;

    if next_time < prev_time {
        return None;
    }
    let segment_duration = next_time - prev_time;
    if segment_duration == 0 {
        return Some(0);
    }

    let dist_from_prev = haversine_distance(prev_stop.lat, prev_stop.lon, lat, lon);
    let dist_to_next = haversine_distance(lat, lon, next_stop.lat, next_stop.lon);

    let fraction = dist_from_prev / (dist_from_prev + dist_to_next);

    let interpolated_sched_secs = prev_time + (segment_duration as f64 * fraction) as u32;

    if let Some(date_str) = &state.assigned_start_date {
        if let Ok(sched_ts_epoch) = get_scheduled_timestamp(date_str, interpolated_sched_secs) {
            return Some(current_time as i64 - sched_ts_epoch as i64);
        }
    }

    None
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

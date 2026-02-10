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
        id: Some(
            state
                .label
                .clone()
                .unwrap_or_else(|| state.vehicle_id.clone()),
        ),
        label: Some(
            state
                .label
                .clone()
                .unwrap_or_else(|| state.vehicle_id.clone()),
        ),
        ..Default::default()
    });

    // Build a map of stop_id -> (visit_index, actual_timestamp) from the vehicle's history
    // We use a greedy monotonic matcher on the position history
    use crate::matcher::proximity::haversine_distance;

    let mut matched_stops: Vec<Option<u64>> = vec![None; stop_times.len()];
    let mut last_matched_sequence_idx = 0;

    // We iterate through all history positions
    let trip_stop_ids: Vec<String> = stop_times.iter().map(|st| st.stop_id.clone()).collect();

    for pos in &state.position_history {
        // We only check stops that are at or ahead of our last matched index
        // To handle lollipops correctly, we must enforce sequence order.
        // We look ahead a few stops (e.g. 5) to allow for missing a stop, but not jumping too far?
        // Actually, strictly monotonic with a small lookahead is safer.
        // But for a lollipop, the physical location of stop N and stop N+K might be same.
        // If we are at index N, and we see location matches N and N+K.
        // If we are definitely at N (based on history), we match N.
        // If we move away and return, we match N+K.

        let mut best_match: Option<(usize, u64)> = None;
        let mut best_dist = 100.0; // Max radius to consider

        // Check upcoming stops (e.g. next 10 stops to allow for some skipped stops)
        let search_end = (last_matched_sequence_idx + 10).min(stop_times.len());

        for idx in last_matched_sequence_idx..search_end {
            let st = &stop_times[idx];
            if let Some(stop) = gtfs.stops.get(&st.stop_id) {
                let dist = haversine_distance(pos.lat, pos.lon, stop.lat, stop.lon);
                if dist < best_dist {
                    best_match = Some((idx, pos.timestamp));
                    best_dist = dist;
                }
            }
        }

        if let Some((idx, ts)) = best_match {
            matched_stops[idx] = Some(ts);
            // If we advanced sequence, update last_matched
            // But we might linger at a stop, so we don't force increment immediately unless we clearly moved?
            // Actually, if we match idx > last_matched, we should probably advance.
            // But if we match idx == last_matched, we just update the timestamp (latest visit).
            // Wait, we want the ARRIVAL time usually, which is the FIRST hit.
            // But for real-time updates, "arrival" is when we got there.
            // Let's keep the FIRST timestamp we matched for a sequence as the arrival.

            if idx >= last_matched_sequence_idx {
                last_matched_sequence_idx = idx;
                // If we haven't recorded a time for this stop yet, use this one (earliest detection)
                if matched_stops[idx].is_none() {
                    matched_stops[idx] = Some(ts);
                }
                // If we accept multiple hits, we might want to know dwell time?
                // For now, let's stick to "first detection" as arrival.
                // Actually, if we are recalculating from history, "first detection" in history order IS arrival.

                // However, the `matched_stops[idx] = Some(ts)` above overwrites.
                // We want the *first* time we hit it in the history.
                if matched_stops[idx].map_or(true, |existing| ts < existing) {
                    matched_stops[idx] = Some(ts);
                }
            }
        }
    }

    // Enforce monotonicity: timestamps must not decrease along the sequence.
    let mut max_ts_seen = 0;
    for matched_ts in matched_stops.iter_mut() {
        if let Some(ts) = *matched_ts {
            if ts < max_ts_seen {
                // Violation: this stop claim to be visited AT 'ts', which is earlier than a previous stop.
                // Invalidate this match.
                *matched_ts = None;
            } else {
                max_ts_seen = ts;
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
            // Calculate delay based on arrival time
            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    propagated_delay = (actual_ts as i64 - sched_ts as i64) as i32;
                }
            }
        }
    }

    // Build stop time updates
    let mut stop_time_updates = Vec::new();

    // Count stop occurrences to determine if sequence is needed
    let mut stop_counts = std::collections::HashMap::new();
    for st in stop_times {
        *stop_counts.entry(&st.stop_id).or_insert(0) += 1;
    }

    for (i, st) in stop_times.iter().enumerate() {
        let mut stu = gtfs_realtime::trip_update::StopTimeUpdate::default();
        if *stop_counts.get(&st.stop_id).unwrap_or(&0) > 1 {
            stu.stop_sequence = Some(st.sequence);
        }
        stu.stop_id = Some(st.stop_id.clone());

        if let Some(actual_ts) = matched_stops[i] {
            // Past stop: use actual arrival time.
            // We can assume departure is same as arrival for past stops if we don't have dwell info,
            // or use specific departure logic if available. For now, use actual_ts for both.
            let mut event = gtfs_realtime::trip_update::StopTimeEvent::default();
            event.time = Some(actual_ts as i64);

            // Calculate delay for past stops
            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    let delay = (actual_ts as i64 - sched_ts as i64) as i32;
                    event.delay = Some(delay);
                }
            }

            stu.arrival = Some(event.clone());
            stu.departure = Some(event);
        } else {
            // Future stop: propagate delay
            // Arrival
            let mut arrival_event = gtfs_realtime::trip_update::StopTimeEvent::default();
            arrival_event.delay = Some(propagated_delay);

            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    arrival_event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            }
            stu.arrival = Some(arrival_event);

            // Departure
            let mut departure_event = gtfs_realtime::trip_update::StopTimeEvent::default();
            departure_event.delay = Some(propagated_delay);

            if let Some(sched_secs) = st.departure_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    departure_event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            } else if let Some(sched_secs) = st.arrival_time_secs {
                // Fallback to arrival time if departure is missing (unlikely given update)
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    departure_event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            }
            stu.departure = Some(departure_event);
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

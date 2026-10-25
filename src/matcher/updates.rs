use crate::gtfs::GtfsData;
use crate::matcher::history::{VehicleState, VehicleStateManager};
use crate::matcher::proximity::haversine_distance;
use gtfs_realtime::FeedMessage;
use std::time::{SystemTime, UNIX_EPOCH};

const DWELL_ENTER_RADIUS_METERS: f64 = 45.0;
const DWELL_EXIT_RADIUS_METERS: f64 = 70.0;
const DWELL_EVIDENCE_WINDOW_SECS: u64 = 20;
const MIN_DWELL_EVIDENCE_SECS: u64 = 8;
const STATIONARY_MEDIAN_SPEED_MPS: f64 = 1.5;
// Do not make live departure predictions from stale GPS observations.
const MAX_POSITION_AGE_SECS: u64 = 45;
// Keep a recently completed trip's final arrival briefly, without inventing
// a departure during the layover for the following trip.
const TERMINAL_ARRIVAL_GRACE_SECS: u64 = 120;

#[derive(Debug, Clone, Copy, Default)]
struct StopMotionEvidence {
    dwelling: bool,
    departure_timestamp: Option<u64>,
}

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
                // Filter out stale trip updates
                let trip = &trip_update.trip;
                if let Some(start_date) = &trip.start_date {
                    use chrono::{Datelike, TimeZone};
                    use chrono_tz::America::Los_Angeles;

                    let now = SystemTime::now();
                    let now_utc = chrono::DateTime::<chrono::Utc>::from(now);
                    let now_la = now_utc.with_timezone(&Los_Angeles);
                    let today = now_la.format("%Y%m%d").to_string();
                    let yesterday = now_la
                        .date_naive()
                        .pred_opt()
                        .unwrap()
                        .format("%Y%m%d")
                        .to_string();
                    let tomorrow = now_la
                        .date_naive()
                        .succ_opt()
                        .unwrap()
                        .format("%Y%m%d")
                        .to_string();

                    if *start_date != today && *start_date != yesterday && *start_date != tomorrow {
                        continue;
                    }
                }

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
    state: &VehicleState,
    trip_id: &str,
    current_time: u64,
) -> Option<gtfs_realtime::TripUpdate> {
    let trip = gtfs.get_trip_by_id(trip_id)?;
    let stop_times = &trip.stop_times;
    let final_stop_idx = stop_times.len().checked_sub(1)?;
    let date_str = state.assigned_start_date.as_ref()?;

    // An old trip assignment must not keep producing predictions as wall time
    // advances after the position feed has stopped updating.
    let latest_position_ts = state.position_history.back()?.timestamp;
    if current_time.saturating_sub(latest_position_ts) > MAX_POSITION_AGE_SECS {
        return None;
    }

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

    // The Viterbi timestamp is an arrival observation. It intentionally stays
    // immutable. Current position history is used separately to infer whether
    // the vehicle is still dwelling or has actually departed.
    let matched_stops = &state.matched_stop_timestamps;
    if matched_stops.len() != stop_times.len() {
        return None;
    }

    // The final stop is an arrival-only event for this trip. After a short
    // grace period, omit the completed trip entirely. Otherwise a bus parked
    // at the terminus can appear to be 40+ minutes late for its old trip.
    if let Some(final_arrival_ts) = matched_stops[final_stop_idx] {
        if current_time.saturating_sub(final_arrival_ts) > TERMINAL_ARRIVAL_GRACE_SECS {
            return None;
        }
    }

    let last_visited_idx = matched_stops.iter().rposition(Option::is_some);

    let current_stop_motion = last_visited_idx.and_then(|idx| {
        let arrival_ts = matched_stops[idx]?;
        let coords = trip.stop_coords.get(idx).copied().flatten()?;
        Some(analyze_stop_motion(state, coords, arrival_ts, current_time))
    });

    let mut propagated_delay = 0i32;
    if let Some(idx) = last_visited_idx {
        if let Some(actual_arrival_ts) = matched_stops[idx] {
            let st = &stop_times[idx];
            let arrival_delay = st
                .arrival_time_secs
                .and_then(|sched_secs| get_scheduled_timestamp(date_str, sched_secs).ok())
                .map(|sched_ts| clamp_delay(actual_arrival_ts as i64 - sched_ts as i64))
                .unwrap_or(0);

            propagated_delay = match current_stop_motion {
                Some(motion) if motion.dwelling => {
                    // While the bus remains at the stop, delay must continue to
                    // grow with wall-clock time. This is a physical lower bound:
                    // the vehicle cannot depart before "now".
                    let scheduled_departure = st
                        .departure_time_secs
                        .or(st.arrival_time_secs)
                        .and_then(|secs| get_scheduled_timestamp(date_str, secs).ok());

                    scheduled_departure
                        .map(|sched_ts| {
                            arrival_delay.max(clamp_delay(current_time as i64 - sched_ts as i64))
                        })
                        .unwrap_or(arrival_delay)
                }
                Some(motion) => motion
                    .departure_timestamp
                    .and_then(|departure_ts| {
                        st.departure_time_secs
                            .or(st.arrival_time_secs)
                            .and_then(|secs| get_scheduled_timestamp(date_str, secs).ok())
                            .map(|sched_ts| clamp_delay(departure_ts as i64 - sched_ts as i64))
                    })
                    .unwrap_or(arrival_delay),
                None => arrival_delay,
            };
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

        if let Some(actual_arrival_ts) = matched_stops[i] {
            // Arrival is an immutable observed event. Do not slide it forward
            // merely because the bus continues to sit at the stop.
            let mut arrival_event = gtfs_realtime::trip_update::StopTimeEvent::default();
            arrival_event.time = Some(actual_arrival_ts as i64);

            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    arrival_event.delay =
                        Some(clamp_delay(actual_arrival_ts as i64 - sched_ts as i64));
                }
            }
            stu.arrival = Some(arrival_event);

            let motion = trip
                .stop_coords
                .get(i)
                .copied()
                .flatten()
                .map(|coords| analyze_stop_motion(state, coords, actual_arrival_ts, current_time))
                .unwrap_or_default();

            // GTFS trips end at their final stop. A departure from that stop
            // belongs to another trip, not an indefinitely growing layover.
            if i != final_stop_idx {
                let is_current_stop = last_visited_idx == Some(i);
                let mut departure_event = gtfs_realtime::trip_update::StopTimeEvent::default();

                if is_current_stop && motion.dwelling {
                // We do not know the eventual departure yet. Publish the
                // earliest physically possible departure (now) and the matching
                // delay lower bound. This value advances every feed refresh.
                let sched_secs = st.departure_time_secs.or(st.arrival_time_secs);
                if let Some(sched_secs) = sched_secs {
                    if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                        let departure_delay = propagated_delay
                            .max(clamp_delay(current_time as i64 - sched_ts as i64));
                        departure_event.delay = Some(departure_delay);
                        departure_event.time = Some(sched_ts as i64 + departure_delay as i64);
                    }
                }
            } else if let Some(departure_ts) = motion.departure_timestamp {
                departure_event.time = Some(departure_ts as i64);
                let sched_secs = st.departure_time_secs.or(st.arrival_time_secs);
                if let Some(sched_secs) = sched_secs {
                    if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                        departure_event.delay =
                            Some(clamp_delay(departure_ts as i64 - sched_ts as i64));
                    }
                }
            } else {
                // We have an observed arrival but insufficient evidence for a
                // distinct departure. Preserve the old behavior as a fallback.
                departure_event.time = Some(actual_arrival_ts as i64);
                let sched_secs = st.departure_time_secs.or(st.arrival_time_secs);
                if let Some(sched_secs) = sched_secs {
                    if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                        departure_event.delay =
                            Some(clamp_delay(actual_arrival_ts as i64 - sched_ts as i64));
                    }
                }
            }
                stu.departure = Some(departure_event);
            }
        } else {
            // Future stop: propagate the best current departure-delay estimate.
            let mut arrival_event = gtfs_realtime::trip_update::StopTimeEvent::default();
            arrival_event.delay = Some(propagated_delay);

            if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    arrival_event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            }
            stu.arrival = Some(arrival_event);

            let mut departure_event = gtfs_realtime::trip_update::StopTimeEvent::default();
            departure_event.delay = Some(propagated_delay);

            if let Some(sched_secs) = st.departure_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    departure_event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            } else if let Some(sched_secs) = st.arrival_time_secs {
                if let Ok(sched_ts) = get_scheduled_timestamp(date_str, sched_secs) {
                    departure_event.time = Some(sched_ts as i64 + propagated_delay as i64);
                }
            }
            if i != final_stop_idx {
                stu.departure = Some(departure_event);
            }
        }

        stu.schedule_relationship = Some(
            gtfs_realtime::trip_update::stop_time_update::ScheduleRelationship::Scheduled as i32,
        );
        stop_time_updates.push(stu);
    }

    trip_update.stop_time_update = stop_time_updates;
    Some(trip_update)
}

fn analyze_stop_motion(
    state: &VehicleState,
    stop_coords: (f64, f64),
    arrival_timestamp: u64,
    current_time: u64,
) -> StopMotionEvidence {
    let (stop_lat, stop_lon) = stop_coords;
    let positions: Vec<_> = state
        .position_history
        .iter()
        .filter(|p| p.timestamp >= arrival_timestamp)
        .collect();

    if positions.is_empty() {
        return StopMotionEvidence::default();
    }

    // Scan from the matched arrival forward, not just the latest position.
    // A bus may leave a stop, complete a loop, and return to the same location
    // much later. Using only the latest position wrongly treats that return as
    // a continuous 40-minute dwell on the original stop event.
    let mut last_inside_timestamp = None;
    for pos in &positions {
        let distance = haversine_distance(pos.lat, pos.lon, stop_lat, stop_lon);
        if distance <= DWELL_ENTER_RADIUS_METERS {
            last_inside_timestamp = Some(pos.timestamp);
        } else if distance >= DWELL_EXIT_RADIUS_METERS {
            if let Some(departure_timestamp) = last_inside_timestamp {
                return StopMotionEvidence {
                    dwelling: false,
                    departure_timestamp: Some(departure_timestamp),
                };
            }
        }
    }

    // We must have observed this stop and still be inside its enter radius.
    if last_inside_timestamp.is_none() {
        return StopMotionEvidence::default();
    }
    let latest = positions[positions.len() - 1];
    let latest_distance = haversine_distance(latest.lat, latest.lon, stop_lat, stop_lon);
    if latest_distance > DWELL_ENTER_RADIUS_METERS
        || current_time.saturating_sub(latest.timestamp) > MAX_POSITION_AGE_SECS
    {
        return StopMotionEvidence::default();
    }

    let window_start = current_time.saturating_sub(DWELL_EVIDENCE_WINDOW_SECS);
    let recent: Vec<_> = positions
        .iter()
        .copied()
        .filter(|p| p.timestamp >= window_start)
        .collect();

    let observed_duration = recent
        .first()
        .zip(recent.last())
        .map(|(first, last)| last.timestamp.saturating_sub(first.timestamp))
        .unwrap_or(0);

    if recent.len() < 3 || observed_duration < MIN_DWELL_EVIDENCE_SECS {
        return StopMotionEvidence::default();
    }

    let mut speeds = Vec::with_capacity(recent.len().saturating_sub(1));
    for pair in recent.windows(2) {
        let dt = pair[1].timestamp.saturating_sub(pair[0].timestamp);
        if dt == 0 {
            continue;
        }
        let distance = haversine_distance(pair[0].lat, pair[0].lon, pair[1].lat, pair[1].lon);
        speeds.push(distance / dt as f64);
    }

    if speeds.is_empty() {
        return StopMotionEvidence::default();
    }

    speeds.sort_by(|a, b| a.total_cmp(b));
    let median_speed = if speeds.len() % 2 == 0 {
        let upper = speeds.len() / 2;
        (speeds[upper - 1] + speeds[upper]) / 2.0
    } else {
        speeds[speeds.len() / 2]
    };

    StopMotionEvidence {
        dwelling: median_speed <= STATIONARY_MEDIAN_SPEED_MPS,
        departure_timestamp: None,
    }
}

fn clamp_delay(delay: i64) -> i32 {
    delay.clamp(i32::MIN as i64, i32::MAX as i64) as i32
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

#[cfg(test)]
mod tests {
    use super::*;
    use crate::matcher::history::VehicleState;

    fn add_position(state: &mut VehicleState, lat: f64, lon: f64, timestamp: u64) {
        state.add_position(lat, lon, None, timestamp);
    }

    #[test]
    fn detects_sustained_dwell_despite_small_gps_jitter() {
        let mut state = VehicleState::new("bus".to_string());
        let stop = (33.650000, -117.830000);

        add_position(&mut state, 33.650000, -117.830000, 1_000);
        add_position(&mut state, 33.650004, -117.830003, 1_005);
        add_position(&mut state, 33.649997, -117.829998, 1_010);
        add_position(&mut state, 33.650003, -117.830002, 1_015);
        add_position(&mut state, 33.650001, -117.830001, 1_020);

        let evidence = analyze_stop_motion(&state, stop, 1_000, 1_020);
        assert!(evidence.dwelling);
        assert_eq!(evidence.departure_timestamp, None);
    }

    #[test]
    fn departure_uses_last_observation_inside_stop_radius() {
        let mut state = VehicleState::new("bus".to_string());
        let stop = (33.650000, -117.830000);

        add_position(&mut state, 33.650000, -117.830000, 2_000);
        add_position(&mut state, 33.650010, -117.830000, 2_005);
        add_position(&mut state, 33.650200, -117.830000, 2_010);
        add_position(&mut state, 33.650800, -117.830000, 2_015);

        let evidence = analyze_stop_motion(&state, stop, 2_000, 2_015);
        assert!(!evidence.dwelling);
        assert_eq!(evidence.departure_timestamp, Some(2_005));
    }

    #[test]
    fn live_dwell_delay_advances_with_wall_clock() {
        let scheduled_departure = 10_000i64;
        let arrival_delay = 30i32;
        let now = 10_300i64;

        let live_delay = arrival_delay.max(clamp_delay(now - scheduled_departure));
        assert_eq!(live_delay, 300);
    }

    fn test_gtfs(scheduled_secs: &[u32]) -> GtfsData {
        use crate::gtfs::{Stop, StopTime, Trip};
        use std::collections::HashMap;

        let mut stops = HashMap::new();
        let mut times = Vec::new();
        for (idx, &secs) in scheduled_secs.iter().enumerate() {
            let stop_id = format!("stop-{idx}");
            stops.insert(
                stop_id.clone(),
                Stop {
                    id: stop_id.clone(),
                    name: stop_id.clone(),
                    lat: 33.65 + idx as f64 * 0.005,
                    lon: -117.83,
                },
            );
            let time = format!("{:02}:{:02}:{:02}", secs / 3600, (secs / 60) % 60, secs % 60);
            times.push(StopTime {
                stop_id,
                sequence: idx as u32 + 1,
                arrival_time: time.clone(),
                arrival_time_secs: Some(secs),
                departure_time: time,
                departure_time_secs: Some(secs),
            });
        }

        let trip = Trip::new(
            "test-trip".to_string(),
            "test-route".to_string(),
            "Weekday".to_string(),
            "test-block".to_string(),
            times,
            &stops,
        );
        GtfsData::new(stops, vec![trip], HashMap::new(), vec![0], vec![])
    }

    fn assigned_state(matched: Vec<Option<u64>>) -> VehicleState {
        let mut state = VehicleState::new("bus".to_string());
        state.assigned_trip_id = Some("test-trip".to_string());
        state.assigned_start_date = Some("20261022".to_string());
        state.matched_stop_timestamps = matched;
        state
    }

    #[test]
    fn final_stop_reports_arrival_but_never_a_departure() {
        let gtfs = test_gtfs(&[11 * 3600 + 50 * 60, 12 * 3600]);
        let scheduled = get_scheduled_timestamp("20261022", 12 * 3600).unwrap();
        let arrived = scheduled - 60;
        let mut state = assigned_state(vec![None, Some(arrived)]);
        let (lat, lon) = gtfs.trips[0].stop_coords[1].unwrap();
        add_position(&mut state, lat, lon, arrived);
        add_position(&mut state, lat, lon, scheduled + 20);

        let update = generate_single_trip_update(&gtfs, &state, "test-trip", scheduled + 30).unwrap();
        let terminal = &update.stop_time_update[1];
        assert_eq!(terminal.arrival.as_ref().unwrap().delay, Some(-60));
        assert_eq!(terminal.arrival.as_ref().unwrap().time, Some(arrived as i64));
        assert!(terminal.departure.is_none());
    }

    #[test]
    fn completed_trip_does_not_accrue_forty_minute_terminal_delay() {
        let gtfs = test_gtfs(&[11 * 3600 + 50 * 60, 12 * 3600]);
        let scheduled = get_scheduled_timestamp("20261022", 12 * 3600).unwrap();
        let arrived = scheduled - 60;
        let now = scheduled + 40 * 60;
        let mut state = assigned_state(vec![None, Some(arrived)]);
        let (lat, lon) = gtfs.trips[0].stop_coords[1].unwrap();
        add_position(&mut state, lat, lon, arrived);
        add_position(&mut state, lat, lon, now - 10);
        add_position(&mut state, lat, lon, now);

        assert!(generate_single_trip_update(&gtfs, &state, "test-trip", now).is_none());
    }

    #[test]
    fn bus_returning_to_same_stop_does_not_count_as_continuous_dwell() {
        let gtfs = test_gtfs(&[11 * 3600 + 50 * 60, 12 * 3600, 12 * 3600 + 10 * 60]);
        let scheduled = get_scheduled_timestamp("20261022", 12 * 3600).unwrap();
        let arrived = scheduled - 60;
        let now = scheduled + 40 * 60;
        let mut state = assigned_state(vec![None, Some(arrived), None]);
        let (lat, lon) = gtfs.trips[0].stop_coords[1].unwrap();
        add_position(&mut state, lat, lon, arrived);
        add_position(&mut state, lat, lon, arrived + 5);
        add_position(&mut state, lat + 0.005, lon, arrived + 15);
        add_position(&mut state, lat, lon, now - 10);
        add_position(&mut state, lat, lon, now - 5);
        add_position(&mut state, lat, lon, now);

        let evidence = analyze_stop_motion(&state, (lat, lon), arrived, now);
        assert!(!evidence.dwelling);
        assert_eq!(evidence.departure_timestamp, Some(arrived + 5));

        let update = generate_single_trip_update(&gtfs, &state, "test-trip", now).unwrap();
        assert_eq!(update.stop_time_update[1].arrival.as_ref().unwrap().delay, Some(-60));
        assert_eq!(update.stop_time_update[1].departure.as_ref().unwrap().delay, Some(-55));
        assert_eq!(update.stop_time_update[2].arrival.as_ref().unwrap().delay, Some(-55));
        assert!(update.stop_time_update[2].departure.is_none());
    }

    #[test]
    fn real_intermediate_dwell_still_increases_departure_delay() {
        let gtfs = test_gtfs(&[12 * 3600, 12 * 3600 + 10 * 60]);
        let scheduled = get_scheduled_timestamp("20261022", 12 * 3600).unwrap();
        let arrived = scheduled - 60;
        let now = scheduled + 300;
        let mut state = assigned_state(vec![Some(arrived), None]);
        let (lat, lon) = gtfs.trips[0].stop_coords[0].unwrap();
        add_position(&mut state, lat, lon, arrived);
        for offset in [15, 10, 5, 0] {
            add_position(&mut state, lat, lon, now - offset);
        }

        let update = generate_single_trip_update(&gtfs, &state, "test-trip", now).unwrap();
        assert_eq!(update.stop_time_update[0].arrival.as_ref().unwrap().delay, Some(-60));
        assert_eq!(update.stop_time_update[0].departure.as_ref().unwrap().delay, Some(300));
        assert_eq!(update.stop_time_update[1].arrival.as_ref().unwrap().delay, Some(300));
        assert!(update.stop_time_update[1].departure.is_none());
    }

    #[test]
    fn stale_position_does_not_create_live_trip_update() {
        let gtfs = test_gtfs(&[12 * 3600, 12 * 3600 + 10 * 60]);
        let scheduled = get_scheduled_timestamp("20261022", 12 * 3600).unwrap();
        let mut state = assigned_state(vec![Some(scheduled - 60), None]);
        let (lat, lon) = gtfs.trips[0].stop_coords[0].unwrap();
        add_position(&mut state, lat, lon, scheduled - 60);

        assert!(generate_single_trip_update(&gtfs, &state, "test-trip", scheduled + 300).is_none());
    }

}

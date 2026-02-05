use crate::gtfs::{GtfsData, Trip};
use crate::matcher::history::VehicleState;
use crate::matcher::proximity::{find_nearest_stop, is_stop_directional};
use chrono::{Datelike, Local, Timelike};

const TERMINAL_STOP_ID: &str = "157583";
const MIN_STOPS_FOR_ASSIGNMENT: usize = 3;
const CONFIDENCE_THRESHOLD: f64 = 0.5;
const TIME_WINDOW_MINUTES: i32 = 60;

pub fn update_vehicle_state(
    state: &mut VehicleState,
    lat: f64,
    lon: f64,
    bearing: Option<f32>,
    timestamp: u64,
    gtfs: &GtfsData,
) {
    state.add_position(lat, lon, bearing, timestamp);

    if let Some((stop_id, _distance)) = find_nearest_stop(lat, lon, gtfs) {
        let should_record = if let Some(stop) = gtfs.stops.get(&stop_id) {
            match (is_stop_directional(stop), bearing) {
                (Some(is_inbound), Some(b)) => {
                    let bearing_matches = if is_inbound {
                        b > 90.0 && b < 270.0
                    } else {
                        b < 90.0 || b > 270.0
                    };
                    bearing_matches
                }
                _ => true,
            }
        } else {
            true
        };

        if should_record {
            state.record_stop_visit(&stop_id, timestamp);
        }
    }

    if state.visited_stops.len() >= MIN_STOPS_FOR_ASSIGNMENT {
        let now = Local::now();
        let is_weekend = now.weekday().num_days_from_monday() >= 5;
        let current_minutes = (now.hour() * 60 + now.minute()) as i32;

        let candidates = find_candidate_trips(gtfs, is_weekend, current_minutes);

        if let Some((trip_id, confidence)) = find_best_matching_trip(state, &candidates, gtfs) {
            if confidence >= CONFIDENCE_THRESHOLD {
                if state.assigned_trip_id.as_ref() != Some(&trip_id)
                    || confidence > state.trip_confidence
                {
                    state.assigned_trip_id = Some(trip_id);
                    state.assigned_start_date = Some(now.format("%Y%m%d").to_string());
                    state.trip_confidence = confidence;
                }
            }
        }
    }

    if should_transition_trip(state, gtfs) {
        if let Some(current_trip_id) = &state.assigned_trip_id.clone() {
            if let Some(next_trip) = find_next_trip_in_block(current_trip_id, gtfs) {
                state.clear_for_new_trip();
                let now = Local::now();
                state.assigned_trip_id = Some(next_trip.trip_id.clone());
                state.assigned_start_date = Some(now.format("%Y%m%d").to_string());
            }
        }
    }
}

fn find_candidate_trips(gtfs: &GtfsData, is_weekend: bool, current_minutes: i32) -> Vec<&Trip> {
    let trip_indices = gtfs.get_active_trips(is_weekend);

    trip_indices
        .iter()
        .filter_map(|&idx| {
            let trip = &gtfs.trips[idx];
            if let Some(start_mins) = trip.start_time_minutes() {
                let diff = (start_mins as i32 - current_minutes).abs();
                let diff = diff.min(1440 - diff);
                if diff <= TIME_WINDOW_MINUTES
                    || (current_minutes >= start_mins as i32 && diff <= TIME_WINDOW_MINUTES + 110)
                {
                    return Some(trip);
                }
            }
            None
        })
        .collect()
}

fn find_best_matching_trip(
    state: &VehicleState,
    candidates: &[&Trip],
    _gtfs: &GtfsData,
) -> Option<(String, f64)> {
    let mut best_match: Option<(String, f64)> = None;

    for trip in candidates {
        let score = score_trip_match(&state.visited_stops, trip);

        if best_match.as_ref().map(|(_, s)| score > *s).unwrap_or(true) && score > 0.0 {
            best_match = Some((trip.trip_id.clone(), score));
        }
    }

    best_match
}

fn score_trip_match(visited_stops: &[String], trip: &Trip) -> f64 {
    if visited_stops.is_empty() || trip.stop_times.is_empty() {
        return 0.0;
    }

    let trip_stops: Vec<&str> = trip
        .stop_times
        .iter()
        .map(|st| st.stop_id.as_str())
        .collect();

    let mut matches = 0;
    let mut last_trip_idx: Option<usize> = None;
    let mut order_violations = 0;

    for visited in visited_stops {
        if let Some(pos) = trip_stops.iter().position(|&s| s == visited) {
            matches += 1;

            if let Some(prev_idx) = last_trip_idx {
                if pos <= prev_idx {
                    order_violations += 1;
                }
            }
            last_trip_idx = Some(pos);
        }
    }

    if matches == 0 {
        return 0.0;
    }

    let match_ratio = matches as f64 / visited_stops.len() as f64;
    let order_penalty = order_violations as f64 / matches as f64;

    match_ratio * (1.0 - order_penalty * 0.5)
}

fn should_transition_trip(state: &VehicleState, gtfs: &GtfsData) -> bool {
    if state.assigned_trip_id.is_none() {
        return false;
    }

    if let Some(last_visited) = state.visited_stops.last() {
        if last_visited == TERMINAL_STOP_ID {
            if let Some(trip_id) = &state.assigned_trip_id {
                if let Some(trip) = gtfs.trips.iter().find(|t| &t.trip_id == trip_id) {
                    if trip.last_stop_id() == Some(TERMINAL_STOP_ID) {
                        return state.visited_stops.len() >= 10;
                    }
                }
            }
        }
    }

    false
}

fn find_next_trip_in_block<'a>(current_trip_id: &str, gtfs: &'a GtfsData) -> Option<&'a Trip> {
    let current_trip = gtfs.trips.iter().find(|t| t.trip_id == current_trip_id)?;
    let block_trips = gtfs.get_trips_in_block(&current_trip.block_id);

    let current_start = current_trip.start_time_minutes()?;

    block_trips
        .into_iter()
        .filter(|t| {
            t.trip_id != current_trip_id
                && t.start_time_minutes()
                    .map(|s| s > current_start)
                    .unwrap_or(false)
        })
        .min_by_key(|t| t.start_time_minutes().unwrap_or(u32::MAX))
}

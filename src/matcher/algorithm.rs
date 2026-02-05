use crate::gtfs::{GtfsData, Trip};
use crate::matcher::history::VehicleState;
use crate::matcher::proximity::{find_nearest_stop, is_stop_directional};
use chrono::{Datelike, TimeZone, Timelike};
use chrono_tz::America::Los_Angeles;

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

    if state.stop_visit_timestamps.len() >= MIN_STOPS_FOR_ASSIGNMENT {
        let la_time = Los_Angeles.timestamp_opt(timestamp as i64, 0).single();

        if let Some(now) = la_time {
            let is_weekend = now.weekday().num_days_from_monday() >= 5;
            let current_secs = (now.hour() * 3600 + now.minute() * 60 + now.second()) as u32;

            let candidates = find_candidate_trips(gtfs, is_weekend, current_secs);

            if let Some((trip_id, confidence)) =
                find_best_matching_trip(state, &candidates, current_secs)
            {
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
    }

    if should_transition_trip(state, gtfs) {
        if let Some(current_trip_id) = &state.assigned_trip_id.clone() {
            if let Some(next_trip) = find_next_trip_in_block(current_trip_id, gtfs) {
                state.clear_for_new_trip();
                let la_time = Los_Angeles.timestamp_opt(timestamp as i64, 0).single();
                if let Some(now) = la_time {
                    state.assigned_trip_id = Some(next_trip.trip_id.clone());
                    state.assigned_start_date = Some(now.format("%Y%m%d").to_string());
                }
            }
        }
    }
}

fn find_candidate_trips(gtfs: &GtfsData, is_weekend: bool, current_secs: u32) -> Vec<&Trip> {
    let trip_indices = gtfs.get_active_trips(is_weekend);
    let current_minutes = (current_secs / 60) as i32;

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
    current_secs: u32,
) -> Option<(String, f64)> {
    let mut best_match: Option<(String, f64)> = None;

    for trip in candidates {
        let score = score_trip_match(&state.stop_visit_timestamps, trip, current_secs);

        if best_match.as_ref().map(|(_, s)| score > *s).unwrap_or(true) && score > 0.0 {
            best_match = Some((trip.trip_id.clone(), score));
        }
    }

    best_match
}

fn score_trip_match(stop_visits: &[(String, u64)], trip: &Trip, current_secs: u32) -> f64 {
    if stop_visits.is_empty() || trip.stop_times.is_empty() {
        return 0.0;
    }

    let mut total_score = 0.0;
    let mut matches = 0;
    let mut last_trip_idx: Option<usize> = None;
    let mut order_violations = 0;

    for (stop_id, visit_timestamp) in stop_visits {
        if let Some((trip_idx, stop_time)) = trip
            .stop_times
            .iter()
            .enumerate()
            .find(|(_, st)| &st.stop_id == stop_id)
        {
            matches += 1;

            // Check stop order
            if let Some(prev_idx) = last_trip_idx {
                if trip_idx <= prev_idx {
                    order_violations += 1;
                }
            }
            last_trip_idx = Some(trip_idx);

            // Time-based scoring
            if let Some(scheduled_secs) = stop_time.arrival_time_secs {
                let visit_la = Los_Angeles
                    .timestamp_opt(*visit_timestamp as i64, 0)
                    .single();
                if let Some(visit_time) = visit_la {
                    let observed_secs = (visit_time.hour() * 3600
                        + visit_time.minute() * 60
                        + visit_time.second()) as i32;
                    let scheduled_secs = scheduled_secs as i32;

                    // Delta = observed - scheduled. Positive means bus is late, negative means early.
                    let delta = observed_secs - scheduled_secs;

                    // Score based on how close the observed time is to scheduled
                    // Perfect: delta in [-60, 60] (within 1 minute)
                    // Good: delta in [-120, 600] (2 min early to 10 min late)
                    // Poor: larger deviations
                    let time_score = if delta >= -60 && delta <= 60 {
                        1.0
                    } else if delta >= -120 && delta <= 600 {
                        0.8 - (delta.abs() as f64 / 1000.0).min(0.3)
                    } else if delta >= -300 && delta <= 1200 {
                        0.4 - (delta.abs() as f64 / 3000.0).min(0.3)
                    } else {
                        0.0
                    };

                    total_score += time_score;
                } else {
                    total_score += 0.5;
                }
            } else {
                total_score += 0.5;
            }
        }
    }

    if matches == 0 {
        return 0.0;
    }

    let match_ratio = matches as f64 / stop_visits.len() as f64;
    let order_penalty = order_violations as f64 / matches as f64;
    let avg_time_score = total_score / matches as f64;

    // Combine: match ratio, time accuracy, and order correctness
    match_ratio * avg_time_score * (1.0 - order_penalty * 0.5)
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

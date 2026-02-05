use crate::gtfs::{GtfsData, Trip};
use crate::matcher::history::VehicleState;
use crate::matcher::proximity::find_nearest_stop_on_trip;
use chrono::{Datelike, TimeZone, Timelike};
use chrono_tz::America::Los_Angeles;

const MIN_POSITIONS_FOR_MATCHING: usize = 5;
const CONFIDENCE_THRESHOLD: f64 = 0.10;
const TIME_WINDOW_MINUTES: i32 = 90;

pub fn update_vehicle_state(
    state: &mut VehicleState,
    lat: f64,
    lon: f64,
    bearing: Option<f32>,
    timestamp: u64,
    _gtfs: &GtfsData,
) {
    state.add_position(lat, lon, bearing, timestamp);
}

pub fn perform_global_assignment(
    state_manager: &mut crate::matcher::VehicleStateManager,
    gtfs: &GtfsData,
) {
    let vehicle_ids: Vec<String> = state_manager
        .all_states()
        .map(|s| s.vehicle_id.clone())
        .collect();

    let mut all_matches: Vec<(f64, String, String, String)> = Vec::new();

    for vehicle_id in &vehicle_ids {
        if let Some(state) = state_manager.get(vehicle_id) {
            if state.position_history.len() < MIN_POSITIONS_FOR_MATCHING {
                continue;
            }

            let last_pos = match state.position_history.back() {
                Some(p) => p,
                None => continue,
            };

            let la_time = Los_Angeles
                .timestamp_opt(last_pos.timestamp as i64, 0)
                .single();

            if let Some(now) = la_time {
                let is_weekend = now.weekday().num_days_from_monday() >= 5;
                let current_secs = (now.hour() * 3600 + now.minute() * 60 + now.second()) as u32;

                let candidates = find_candidate_trips(gtfs, is_weekend, current_secs);
                println!(
                    "Found {} candidates for vehicle {}",
                    candidates.len(),
                    vehicle_id
                );

                for trip in candidates {
                    let score = score_trip_from_positions(state, trip, gtfs);
                    if score > 0.0 {
                        println!(
                            "Vehicle {} - Trip {} score: {:.3}",
                            vehicle_id, trip.trip_id, score
                        );

                        let final_score =
                            if state.assigned_trip_id.as_deref() == Some(&trip.trip_id) {
                                score + 0.05
                            } else {
                                score
                            };

                        all_matches.push((
                            final_score,
                            vehicle_id.clone(),
                            trip.trip_id.clone(),
                            now.format("%Y%m%d").to_string(),
                        ));
                    }
                }
            }
        }
    }

    all_matches.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    let mut assigned_trips = std::collections::HashSet::new();
    let mut assigned_vehicles = std::collections::HashSet::new();
    let mut final_assignments = std::collections::HashMap::new();

    for (score, vehicle_id, trip_id, start_date) in all_matches {
        if score < CONFIDENCE_THRESHOLD {
            continue;
        }

        if !assigned_vehicles.contains(&vehicle_id) && !assigned_trips.contains(&trip_id) {
            assigned_vehicles.insert(vehicle_id.clone());
            assigned_trips.insert(trip_id.clone());
            final_assignments.insert(vehicle_id, (trip_id, start_date, score));
        }
    }

    for vehicle_id in &vehicle_ids {
        if let Some(state) = state_manager.get_mut(&vehicle_id) {
            if let Some((trip_id, start_date, score)) = final_assignments.get(vehicle_id) {
                if state.assigned_trip_id.as_ref() != Some(trip_id) {
                    println!(
                        "Assigning trip {} to vehicle {} (score: {:.3})",
                        trip_id, vehicle_id, score
                    );
                    state.assigned_trip_id = Some(trip_id.clone());
                    state.assigned_start_date = Some(start_date.clone());
                    state.trip_confidence = *score;
                } else {
                    state.trip_confidence = *score;
                }
            } else if state.assigned_trip_id.is_some() {
                println!(
                    "Vehicle {} lost its assignment (no valid match found or trip taken)",
                    vehicle_id
                );
                state.assigned_trip_id = None;
                state.assigned_start_date = None;
                state.trip_confidence = 0.0;
            }
        }
    }

    println!("Processed {} vehicles", vehicle_ids.len());
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
                    || (current_minutes >= start_mins as i32 && diff <= TIME_WINDOW_MINUTES + 60)
                {
                    return Some(trip);
                }
            }
            None
        })
        .collect()
}

fn score_trip_from_positions(state: &VehicleState, trip: &Trip, gtfs: &GtfsData) -> f64 {
    if state.position_history.is_empty() || trip.stop_times.is_empty() {
        return 0.0;
    }

    let trip_stop_ids: Vec<String> = trip
        .stop_times
        .iter()
        .map(|st| st.stop_id.clone())
        .collect();

    struct StopMatch {
        stop_idx: usize,
        position_timestamp: u64,
        scheduled_secs: Option<u32>,
    }

    let mut matches: Vec<StopMatch> = Vec::new();

    for pos in &state.position_history {
        if let Some((stop_idx, _stop_id, _dist)) =
            find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs)
        {
            if matches.last().map(|m| m.stop_idx) != Some(stop_idx) {
                let scheduled_secs = trip
                    .stop_times
                    .get(stop_idx)
                    .and_then(|st| st.arrival_time_secs);
                matches.push(StopMatch {
                    stop_idx,
                    position_timestamp: pos.timestamp,
                    scheduled_secs,
                });
            }
        }
    }

    if matches.is_empty() {
        return 0.0;
    }

    let unique_stops = matches.len();

    // Order score: longest increasing run
    let mut longest_increasing = 1;
    let mut current_run = 1;
    for i in 1..matches.len() {
        if matches[i].stop_idx > matches[i - 1].stop_idx {
            current_run += 1;
            longest_increasing = longest_increasing.max(current_run);
        } else {
            current_run = 1;
        }
    }
    let order_score = longest_increasing as f64 / unique_stops as f64;

    // Time score: how well do position timestamps match scheduled times?
    let mut time_scores: Vec<f64> = Vec::new();
    for m in &matches {
        if let Some(scheduled_secs) = m.scheduled_secs {
            let pos_la = Los_Angeles
                .timestamp_opt(m.position_timestamp as i64, 0)
                .single();
            if let Some(pos_time) = pos_la {
                let observed_secs =
                    (pos_time.hour() * 3600 + pos_time.minute() * 60 + pos_time.second()) as i32;
                let scheduled = scheduled_secs as i32;
                let delta = observed_secs - scheduled;

                // Score based on time difference
                // Perfect: within 2 minutes
                // Good: within 10 minutes
                // Acceptable: within 20 minutes
                // Bad: beyond that
                let ts = if delta.abs() <= 120 {
                    1.0
                } else if delta.abs() <= 600 {
                    0.7
                } else if delta.abs() <= 1200 {
                    0.3
                } else {
                    0.0
                };
                time_scores.push(ts);
            }
        }
    }

    let avg_time_score = if time_scores.is_empty() {
        0.5
    } else {
        time_scores.iter().sum::<f64>() / time_scores.len() as f64
    };

    let coverage = unique_stops as f64 / trip.stop_times.len() as f64;

    // Combine: spatial matching, order, time accuracy, coverage
    let raw_score =
        (unique_stops as f64).sqrt() * order_score * avg_time_score * (0.3 + 0.7 * coverage);

    raw_score.min(1.0)
}

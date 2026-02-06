use crate::gtfs::{GtfsData, Trip};
use crate::matcher::history::VehicleState;
use crate::matcher::proximity::{find_nearest_stop_on_trip, haversine_distance};
use chrono::{Datelike, TimeZone, Timelike};
use chrono_tz::America::Los_Angeles;
use std::collections::{HashMap, HashSet};
use std::time::SystemTime;

const MIN_POSITIONS_FOR_MATCHING: usize = 5;
const CONFIDENCE_THRESHOLD: f64 = 0.05;
const TIME_WINDOW_MINUTES: i32 = 90;

const TERMINUS_LAT: f64 = 33.656831556;
const TERMINUS_LON: f64 = -117.732860708;
const TERMINUS_RADIUS_METERS: f64 = 80.0;

// Dynamic thresholds will be used instead of fixed constants where possible
// const LATE_STOP_THRESHOLD: usize = 70;
const EARLY_STOP_THRESHOLD: usize = 15;

const TRANSITION_BONUS: f64 = 1.5;
// const PREVIOUS_TRIP_PENALTY: f64 = 0.8;

#[derive(Debug, Clone, Copy, PartialEq)]
enum TerminusVisitType {
    Arriving,
    Departing,
    Unknown,
}

#[derive(Debug)]
struct TerminusVisit {
    position_idx: usize,
    _timestamp: u64,
    visit_type: TerminusVisitType,
}

struct RawTerminusVisit {
    position_idx: usize,
    timestamp: u64,
}

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

    // First pass: Standard assignment with segmentation
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

                let mut candidates = find_candidate_trips(gtfs, is_weekend, current_secs);

                // Include block neighbors if we have an assigned trip
                if let Some(assigned_id) = &state.assigned_trip_id {
                    // Current assigned
                    if let Some(curr) = gtfs.get_trip_by_id(assigned_id) {
                        if !candidates.iter().any(|t| t.trip_id == curr.trip_id) {
                            candidates.push(curr);
                        }
                    }

                    if let Some(prev) = gtfs.get_previous_trip_in_block(assigned_id) {
                        if !candidates.iter().any(|t| t.trip_id == prev.trip_id) {
                            candidates.push(prev);
                        }
                    }
                    if let Some(next) = gtfs.get_next_trip_in_block(assigned_id) {
                        if !candidates.iter().any(|t| t.trip_id == next.trip_id) {
                            candidates.push(next);
                        }
                    }
                }

                println!(
                    "Found {} candidates for vehicle {}",
                    candidates.len(),
                    vehicle_id
                );

                // 1. Find all raw visits to terminus (trip-independent)
                let raw_terminus_visits = find_raw_terminus_visits(state);

                for trip in &candidates {
                    // Classify visits specific to THIS trip
                    let terminus_visits =
                        classify_terminus_visits_for_trip(state, trip, &raw_terminus_visits, gtfs);
                    let segment_start = find_segment_boundary(&terminus_visits);

                    let (score, transition_detected) = score_trip_with_segmentation(
                        state,
                        trip,
                        gtfs,
                        segment_start,
                        &terminus_visits,
                    );

                    if score > 0.0 {
                        let mut final_score = score;

                        // Stability bonus for staying on same trip
                        if state.assigned_trip_id.as_deref() == Some(&trip.trip_id) {
                            final_score += 0.05;
                        }

                        // Transition bonus
                        if transition_detected {
                            if let Some(prev_in_block) =
                                gtfs.get_previous_trip_in_block(&trip.trip_id)
                            {
                                if state.assigned_trip_id.as_deref() == Some(&prev_in_block.trip_id)
                                    || state.previous_trip_id.as_deref()
                                        == Some(&prev_in_block.trip_id)
                                {
                                    println!(
                                        "Vehicle {} - Transition detected from {} to {}",
                                        vehicle_id, prev_in_block.trip_id, trip.trip_id
                                    );
                                    final_score *= TRANSITION_BONUS;
                                }
                            }
                        }

                        // Penalize previous trip in block if we see early stops of next trip
                        if let Some(next_in_block) = gtfs.get_next_trip_in_block(&trip.trip_id) {
                            if history_shows_early_stops(state, &next_in_block, gtfs, segment_start)
                            {
                                println!(
                                    "Vehicle {} - Penalizing {} (history shows next trip {})",
                                    vehicle_id, trip.trip_id, next_in_block.trip_id
                                );
                            }
                        }

                        println!(
                            "Vehicle {} - Trip {} score: {:.3}{}",
                            vehicle_id,
                            trip.trip_id,
                            final_score,
                            if transition_detected {
                                " [TRANSITION]"
                            } else {
                                ""
                            }
                        );

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

    let mut assigned_trips = HashSet::new();
    let mut assigned_vehicles = HashSet::new();
    let mut final_assignments = HashMap::new();
    let mut fallback_routes = HashMap::new();

    // Apply confident matches
    for (score, vehicle_id, trip_id, start_date) in &all_matches {
        if assigned_vehicles.contains(vehicle_id) {
            continue;
        }

        if *score >= CONFIDENCE_THRESHOLD {
            if !assigned_trips.contains(trip_id) {
                assigned_vehicles.insert(vehicle_id.clone());
                assigned_trips.insert(trip_id.clone());

                let route_id = gtfs.get_trip_by_id(trip_id).map(|t| t.route_id.clone());
                final_assignments.insert(
                    vehicle_id.clone(),
                    (trip_id.clone(), route_id, start_date.clone(), *score),
                );
            }
        } else {
            if !fallback_routes.contains_key(vehicle_id) {
                if let Some(trip) = gtfs.get_trip_by_id(trip_id) {
                    fallback_routes.insert(vehicle_id.clone(), trip.route_id.clone());
                }
            }
        }
    }

    // Spatial Fallback Pass for unassigned vehicles
    for vehicle_id in &vehicle_ids {
        if assigned_vehicles.contains(vehicle_id) {
            continue;
        }

        if let Some(state) = state_manager.get(vehicle_id) {
            if let Some(last_pos) = state.position_history.back() {
                // Find nearest stops
                let mut valid_fallback: Option<(&Trip, f64)> = None;

                let la_time = Los_Angeles
                    .timestamp_opt(last_pos.timestamp as i64, 0)
                    .single();

                if let Some(now) = la_time {
                    let current_secs =
                        (now.hour() * 3600 + now.minute() * 60 + now.second()) as u32;
                    let is_weekend = now.weekday().num_days_from_monday() >= 5;

                    // Optimization: Filter active trips first, then check geometry?
                    // Or finding nearby stops is cheap?
                    // Let's find stops within 500m
                    let nearby_stop_ids: Vec<&String> = gtfs
                        .stops
                        .values()
                        .filter(|s| {
                            haversine_distance(last_pos.lat, last_pos.lon, s.lat, s.lon) < 500.0
                        })
                        .map(|s| &s.id)
                        .collect();

                    if !nearby_stop_ids.is_empty() {
                        let active_trip_indices = gtfs.get_active_trips(is_weekend);
                        for &idx in active_trip_indices {
                            let trip = &gtfs.trips[idx];
                            // Skip if already assigned
                            if assigned_trips.contains(&trip.trip_id) {
                                continue;
                            }

                            // Check if trip visits any nearby stop within time window
                            for st in &trip.stop_times {
                                if nearby_stop_ids.contains(&&st.stop_id) {
                                    // Check time
                                    if let Some(arrival) = st.arrival_time_secs {
                                        let diff = (arrival as i32 - current_secs as i32).abs();
                                        // +/- 20 minutes = 1200 seconds
                                        if diff <= 1200 {
                                            // Found a match!
                                            // Run full scoring to verify history sequence
                                            // Since we have no segment info (fallback), treat as arriving (full history)
                                            // or try to segment?
                                            // For fallback, simpler might be better, but let's try segmentation.

                                            let raw_terminus_visits =
                                                find_raw_terminus_visits(state);
                                            let terminus_visits = classify_terminus_visits_for_trip(
                                                state,
                                                trip,
                                                &raw_terminus_visits,
                                                gtfs,
                                            );
                                            let segment_start =
                                                find_segment_boundary(&terminus_visits);

                                            let (score, _) = score_trip_with_segmentation(
                                                state,
                                                trip,
                                                gtfs,
                                                segment_start,
                                                &terminus_visits,
                                            );

                                            // If score is decent (even if low confidence), take it
                                            if score > 0.05 {
                                                // Use same low threshold
                                                if valid_fallback.map_or(true, |(_, s)| score > s) {
                                                    valid_fallback = Some((trip, score));
                                                }
                                            }
                                        }
                                    }
                                }
                            }
                        }
                    }
                }

                if let Some((trip, score)) = valid_fallback {
                    println!(
                        "Vehicle {} - Spatial Fallback assigned {} (score {:.2})",
                        vehicle_id, trip.trip_id, score
                    );
                    assigned_vehicles.insert(vehicle_id.clone());
                    assigned_trips.insert(trip.trip_id.clone());
                    let start_date = la_time
                        .map(|t| t.format("%Y%m%d").to_string())
                        .unwrap_or("20240101".to_string());
                    final_assignments.insert(
                        vehicle_id.clone(),
                        (
                            trip.trip_id.clone(),
                            Some(trip.route_id.clone()),
                            start_date,
                            score,
                        ),
                    );
                }
            }
        }
    }

    // Apply assignments to state
    for vehicle_id in &vehicle_ids {
        if let Some(state) = state_manager.get_mut(vehicle_id) {
            let fallback_route = fallback_routes.get(vehicle_id).cloned();

            if let Some((trip_id, route_id, start_date, score)) = final_assignments.get(vehicle_id)
            {
                if state.assigned_trip_id.as_ref() != Some(trip_id) {
                    let is_block_transition =
                        state.assigned_trip_id.as_ref().map_or(false, |current| {
                            gtfs.get_next_trip_in_block(current)
                                .map(|next| &next.trip_id == trip_id)
                                .unwrap_or(false)
                        });

                    if is_block_transition {
                        println!(
                            "Block transition: {} -> {} for vehicle {} (score: {:.3})",
                            state.assigned_trip_id.as_deref().unwrap_or("none"),
                            trip_id,
                            vehicle_id,
                            score
                        );
                        let timestamp = state
                            .position_history
                            .back()
                            .map(|p| p.timestamp)
                            .unwrap_or(0);
                        state.transition_to_new_trip(trip_id.as_str(), timestamp);
                        state.route_id = route_id.clone();
                        state.assigned_start_date = Some(start_date.clone());
                        state.trip_confidence = *score;
                    } else {
                        println!(
                            "Assigning trip {} to vehicle {} (score: {:.3})",
                            trip_id, vehicle_id, score
                        );
                        state.assigned_trip_id = Some(trip_id.clone());
                        state.route_id = route_id.clone();
                        state.assigned_start_date = Some(start_date.clone());
                        state.trip_confidence = *score;
                    }
                } else {
                    state.trip_confidence = *score;
                    if state.route_id.is_none() {
                        state.route_id = route_id.clone();
                    }
                }
            } else {
                // No assignment
                if state.assigned_trip_id.is_some() {
                    println!(
                        "Vehicle {} lost its assignment (no valid match found)",
                        vehicle_id
                    );
                    state.assigned_trip_id = None;
                    state.assigned_start_date = None;
                    state.trip_confidence = 0.0;

                    state.route_id = fallback_route;
                    if let Some(r) = &state.route_id {
                        println!("Vehicle {} fallback to route {}", vehicle_id, r);
                    }
                } else {
                    if let Some(r) = fallback_route {
                        if state.route_id.as_ref() != Some(&r) {
                            state.route_id = Some(r.clone());
                            println!("Vehicle {} assigned fallback route {}", vehicle_id, r);
                        }
                    }
                }
            }
        }
    }

    println!("Processed {} vehicles", vehicle_ids.len());
}

/// Find all positions where the vehicle was at or near the terminus (location only)
fn find_raw_terminus_visits(state: &VehicleState) -> Vec<RawTerminusVisit> {
    let mut visits = Vec::new();

    for (idx, pos) in state.position_history.iter().enumerate() {
        let dist = haversine_distance(pos.lat, pos.lon, TERMINUS_LAT, TERMINUS_LON);

        if dist <= TERMINUS_RADIUS_METERS {
            visits.push(RawTerminusVisit {
                position_idx: idx,
                timestamp: pos.timestamp,
            });
        }
    }
    visits
}

/// Classify whether a terminus visit is arriving (end of trip) or departing (start of trip)
/// SPECIFIC to a candidate trip
fn classify_terminus_visits_for_trip(
    state: &VehicleState,
    trip: &Trip,
    raw_visits: &[RawTerminusVisit],
    gtfs: &GtfsData,
) -> Vec<TerminusVisit> {
    let mut classified = Vec::new();

    // Dynamic threshold: e.g. last 5 stops of the trip
    let late_threshold_idx = trip.stop_times.len().saturating_sub(5);

    let trip_stop_ids: Vec<String> = trip
        .stop_times
        .iter()
        .map(|st| st.stop_id.clone())
        .collect();

    for raw in raw_visits {
        // Look at positions before this terminus visit
        let before_late = state
            .position_history
            .iter()
            .take(raw.position_idx)
            .rev()
            .take(5)
            .filter_map(|pos| {
                find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs)
                    .map(|(idx, _, _)| idx)
            })
            // Check if matched index is 'late' in the trip (near end)
            .any(|idx| idx >= late_threshold_idx);

        // Look at positions after this terminus visit
        let after_early = state
            .position_history
            .iter()
            .skip(raw.position_idx + 1)
            .take(5)
            .filter_map(|pos| {
                find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs)
                    .map(|(idx, _, _)| idx)
            })
            // Check if matched index is 'early' in the trip (near start)
            .any(|idx| idx > 0 && idx <= EARLY_STOP_THRESHOLD);

        let visit_type = match (before_late, after_early) {
            (true, false) => TerminusVisitType::Arriving,
            (false, true) => TerminusVisitType::Departing,
            (true, true) => TerminusVisitType::Departing, // Transition happening
            (false, false) => TerminusVisitType::Unknown,
        };

        if visit_type != TerminusVisitType::Unknown {
            classified.push(TerminusVisit {
                position_idx: raw.position_idx,
                _timestamp: raw.timestamp,
                visit_type,
            });
        }
    }

    classified
}

/// Find the segment boundary: the index in position_history after which we should score
fn find_segment_boundary(terminus_visits: &[TerminusVisit]) -> Option<usize> {
    // Find the most recent departing terminus visit
    terminus_visits
        .iter()
        .rev()
        .find(|v| v.visit_type == TerminusVisitType::Departing)
        .map(|v| v.position_idx)
}

/// Score a trip using segmented history with greedy monotonic matching
fn score_trip_with_segmentation(
    state: &VehicleState,
    trip: &Trip,
    gtfs: &GtfsData,
    segment_start: Option<usize>,
    terminus_visits: &[TerminusVisit],
) -> (f64, bool) {
    if state.position_history.is_empty() || trip.stop_times.is_empty() {
        return (0.0, false);
    }

    let start_idx = segment_start.unwrap_or(0);
    // If we have very little history to work with, we can't be confident
    if state.position_history.len() - start_idx < MIN_POSITIONS_FOR_MATCHING {
        return (0.0, false);
    }

    let stop_times = &trip.stop_times;

    // Matched stops: Vec of (stop_index, timestamp_of_visit)
    // We use the same greedy monotonic logic as in updates.rs
    let mut matched_stops: Vec<Option<u64>> = vec![None; stop_times.len()];
    let mut last_matched_sequence_idx = 0;

    let history_slice: Vec<_> = state.position_history.iter().skip(start_idx).collect();

    // Need date string for scheduled timestamp calculation.
    // If not assigned yet, use today in LA.
    let date_str = if let Some(d) = &state.assigned_start_date {
        d.clone()
    } else {
        // Fallback to today
        let now = SystemTime::now();
        // We really want the date of the position timestamps usually, but
        // for candidate scoring we are usually looking at "now" or recent.
        // Let's assume the trip runs "today" relative to the vehicle positions.
        // A better approx is taking the last position timestamp.
        if let Some(last_pos) = history_slice.last() {
            Los_Angeles
                .timestamp_opt(last_pos.timestamp as i64, 0)
                .single()
                .map(|dt| dt.format("%Y%m%d").to_string())
                .unwrap_or_else(|| "20240101".to_string())
        } else {
            "20240101".to_string()
        }
    };

    for pos in &history_slice {
        let mut best_match: Option<(usize, u64)> = None;
        let mut best_dist = 100.0; // Max radius matches updates.rs logic approximately

        // Search lookahead
        let search_end = (last_matched_sequence_idx + 10).min(stop_times.len());

        for idx in last_matched_sequence_idx..search_end {
            let st = &stop_times[idx];
            if let Some(stop) = gtfs.stops.get(&st.stop_id) {
                let dist = haversine_distance(pos.lat, pos.lon, stop.lat, stop.lon);
                // 80 meters radius, consistent with typical bus stop radius
                if dist < 80.0 && dist < best_dist {
                    best_match = Some((idx, pos.timestamp));
                    best_dist = dist;
                }
            }
        }

        if let Some((idx, ts)) = best_match {
            if idx >= last_matched_sequence_idx {
                last_matched_sequence_idx = idx;
                // Capture the FIRST time we hit this stop in the sequence (arrival)
                if matched_stops[idx].is_none() {
                    matched_stops[idx] = Some(ts);
                }
            }
        }
    }

    let unique_stops_matched = matched_stops.iter().filter(|m| m.is_some()).count();

    if unique_stops_matched == 0 {
        return (0.0, false);
    }

    // Calculate time conformance
    let mut total_delta_sum = 0.0;
    let mut delta_count = 0;
    let mut weighted_score_sum = 0.0;
    let mut total_weight = 0.0;

    // Helper to get scheduled timestamp
    // We duplicate the logic from updates.rs effectively here since it's small
    let get_sched_ts = |secs: u32| -> Option<i64> {
        use chrono::NaiveDate;
        if let Ok(date) = NaiveDate::parse_from_str(&date_str, "%Y%m%d") {
            if let Some(naive_dt) = date.and_hms_opt(0, 0, 0) {
                if let Some(base_dt) = Los_Angeles.from_local_datetime(&naive_dt).single() {
                    let sched_dt = base_dt + chrono::Duration::seconds(secs as i64);
                    return Some(sched_dt.timestamp());
                }
            }
        }
        None
    };

    let latest_timestamp = history_slice.last().map(|p| p.timestamp).unwrap_or(0);

    for (idx, ts_opt) in matched_stops.iter().enumerate() {
        if let Some(actual_ts) = ts_opt {
            let st = &stop_times[idx];
            if let Some(sched_secs) = st.arrival_time_secs {
                if let Some(sched_ts) = get_sched_ts(sched_secs) {
                    let delta = *actual_ts as i64 - sched_ts;

                    total_delta_sum += delta as f64;
                    delta_count += 1;

                    // Recency weight
                    let seconds_ago = if latest_timestamp > *actual_ts {
                        latest_timestamp - *actual_ts
                    } else {
                        0
                    };

                    let weight = if seconds_ago <= 300 {
                        1.0
                    } else if seconds_ago <= 1200 {
                        1.0 - (0.8 * (seconds_ago - 300) as f64 / 900.0)
                    } else {
                        0.2
                    };

                    // Time Match Score
                    // Similar logic to previous:
                    // -5m to +10m: Good
                    // -10m to +20m: Okay
                    // -20m to +30m: Poor
                    let ts_score = if delta >= -300 && delta <= 600 {
                        1.0
                    } else if delta >= -600 && delta <= 1200 {
                        0.7
                    } else if delta >= -1200 && delta <= 1800 {
                        0.3
                    } else {
                        0.0
                    };

                    weighted_score_sum += ts_score * weight;
                    total_weight += weight;
                }
            }
        }
    }

    // Excessive delay check
    if delta_count > 0 {
        let avg_delta = total_delta_sum / delta_count as f64;
        // Strictly penalize trips that are on average > 30 mins late or > 30 mins early
        if avg_delta.abs() > 1800.0 {
            println!(
                "Trip {} disqualified due to excessive average delay: {:.1}s",
                trip.trip_id, avg_delta
            );
            return (0.0, false);
        }
    }

    let avg_time_score = if total_weight > 0.0 {
        weighted_score_sum / total_weight
    } else {
        0.0 // No valid time comparisons means we can't verify this trip
    };

    let coverage = unique_stops_matched as f64 / stop_times.len() as f64;

    // Check for transition signature using existing helper
    let transition_detected = detect_transition_signature(state, trip, gtfs, terminus_visits);

    // Consider early stop presence in recent history
    // We can check if matched_stops contain early indices
    let has_early_stops = matched_stops
        .iter()
        .enumerate()
        .any(|(i, m)| m.is_some() && i <= EARLY_STOP_THRESHOLD);
    let early_bonus = if has_early_stops && segment_start.is_some() {
        1.2
    } else {
        1.0
    };

    // Calculate final score
    let raw_score = (unique_stops_matched as f64).sqrt()
        * avg_time_score
        * (0.3 + 0.7 * coverage)
        * early_bonus;

    (raw_score.min(1.0), transition_detected)
}

/// Detect if position history shows a transition from previous trip to this trip
fn detect_transition_signature(
    state: &VehicleState,
    trip: &Trip,
    gtfs: &GtfsData,
    terminus_visits: &[TerminusVisit],
) -> bool {
    // We need at least one terminus visit
    if terminus_visits.is_empty() {
        return false;
    }

    let trip_stop_ids: Vec<String> = trip
        .stop_times
        .iter()
        .map(|st| st.stop_id.clone())
        .collect();

    // Find the most recent terminus visit
    let last_terminus = terminus_visits.last().unwrap();

    // Dynamic threshold: e.g. last 5 stops of the trip
    let late_threshold_idx = trip.stop_times.len().saturating_sub(5);

    // Check if positions before terminus matched late stops
    let pre_terminus_late = state
        .position_history
        .iter()
        .take(last_terminus.position_idx)
        .rev()
        .take(10)
        .filter_map(|pos| {
            find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs).map(|(idx, _, _)| idx)
        })
        .any(|idx| idx >= late_threshold_idx);

    // Check if positions after terminus match early stops
    let post_terminus_early = state
        .position_history
        .iter()
        .skip(last_terminus.position_idx + 1)
        .take(10)
        .filter_map(|pos| {
            find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs).map(|(idx, _, _)| idx)
        })
        .any(|idx| idx > 0 && idx <= EARLY_STOP_THRESHOLD);

    pre_terminus_late && post_terminus_early
}

/// Check if the position history shows early stops of a given trip
fn history_shows_early_stops(
    state: &VehicleState,
    trip: &Trip,
    gtfs: &GtfsData,
    segment_start: Option<usize>,
) -> bool {
    let trip_stop_ids: Vec<String> = trip
        .stop_times
        .iter()
        .map(|st| st.stop_id.clone())
        .collect();

    let start_idx = segment_start.unwrap_or(0);

    state
        .position_history
        .iter()
        .skip(start_idx)
        .filter_map(|pos| {
            find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs).map(|(idx, _, _)| idx)
        })
        .any(|idx| idx > 0 && idx <= EARLY_STOP_THRESHOLD)
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

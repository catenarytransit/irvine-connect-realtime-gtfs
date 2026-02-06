use crate::gtfs::{GtfsData, Trip};
use crate::matcher::history::VehicleState;
use crate::matcher::proximity::{find_nearest_stop_on_trip, haversine_distance};
use chrono::{Datelike, TimeZone, Timelike};
use chrono_tz::America::Los_Angeles;

const MIN_POSITIONS_FOR_MATCHING: usize = 5;
const CONFIDENCE_THRESHOLD: f64 = 0.05;
const TIME_WINDOW_MINUTES: i32 = 90;

const TERMINUS_STOP_ID: &str = "157583";
const TERMINUS_LAT: f64 = 33.656831556;
const TERMINUS_LON: f64 = -117.732860708;
const TERMINUS_RADIUS_METERS: f64 = 80.0;

const LATE_STOP_THRESHOLD: usize = 70;
const EARLY_STOP_THRESHOLD: usize = 15;

const TRANSITION_BONUS: f64 = 1.5;
const PREVIOUS_TRIP_PENALTY: f64 = 0.8;



#[derive(Debug, Clone, Copy, PartialEq)]
enum TerminusVisitType {
    Arriving,
    Departing,
    Unknown,
}

#[derive(Debug)]
struct TerminusVisit {
    position_idx: usize,
    timestamp: u64,
    visit_type: TerminusVisitType,
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

                let terminus_visits = find_terminus_visits(state, gtfs);
                let segment_start = find_segment_boundary(&terminus_visits, state, gtfs);

                // Infer current trip from pre-segment history
                // If pre-segment matches late stops of trip T-1, the current trip is T
                let inferred_trip = if let Some(seg_start) = segment_start {
                    infer_trip_from_pre_segment(state, gtfs, seg_start, is_weekend)
                } else {
                    None
                };

                if let Some(ref inferred) = inferred_trip {
                    println!(
                        "Vehicle {} - Inferred current trip {} from pre-segment history",
                        vehicle_id, inferred.trip_id
                    );
                    if !candidates.iter().any(|t| t.trip_id == inferred.trip_id) {
                        candidates.push(inferred);
                    }
                }

                for trip in &candidates {
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

                        // Strong bonus if this trip was inferred from pre-segment history + departure time
                        if inferred_trip
                            .as_ref()
                            .map_or(false, |inf| inf.trip_id == trip.trip_id)
                        {
                            println!(
                                "Vehicle {} - Boosting {} (inferred from pre-segment + departure time)",
                                vehicle_id, trip.trip_id
                            );
                            final_score *= 2.0;
                        }

                        // Transition bonus when we detect movement from previous trip to this one
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

    let mut assigned_trips = std::collections::HashSet::new();
    let mut assigned_vehicles = std::collections::HashSet::new();
    let mut final_assignments = std::collections::HashMap::new();
    let mut fallback_routes = std::collections::HashMap::new();

    for (score, vehicle_id, trip_id, start_date) in all_matches {
        // If we already have a confident assignment for this vehicle, skip
        if assigned_vehicles.contains(&vehicle_id) {
            continue;
        }

        if score >= CONFIDENCE_THRESHOLD {
            if !assigned_trips.contains(&trip_id) {
                assigned_vehicles.insert(vehicle_id.clone());
                assigned_trips.insert(trip_id.clone());
                
                let route_id = gtfs.get_trip_by_id(&trip_id).map(|t| t.route_id.clone());
                final_assignments.insert(vehicle_id, (trip_id, route_id, start_date, score));
            }
        } else {
             // Best match is below threshold. Store as fallback route if we haven't seen this vehicle 
             // in this loop yet (since it's sorted by score, first time is best match).
             // Since we check assigned_vehicles above allowing continue, and we are in else branch,
             // we successfully missed the confidence check.
             // But valid candidates might be multiple. 
             // We only want the best one for fallback.
             if !fallback_routes.contains_key(&vehicle_id) {
                 if let Some(trip) = gtfs.get_trip_by_id(&trip_id) {
                     fallback_routes.insert(vehicle_id.clone(), trip.route_id.clone());
                 }
             }
        }
    }

    for vehicle_id in &vehicle_ids {
        if let Some(state) = state_manager.get_mut(&vehicle_id) {
            let fallback_route = fallback_routes.get(vehicle_id).cloned();

            if let Some((trip_id, route_id, start_date, score)) = final_assignments.get(vehicle_id) {
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
                        state.transition_to_new_trip(trip_id, timestamp);
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
                    // Ensure route_id is set if it was missing
                    if state.route_id.is_none() {
                         state.route_id = route_id.clone();
                    }
                }
            } else {
                // No confident assignment found
                if state.assigned_trip_id.is_some() {
                    println!(
                        "Vehicle {} lost its assignment (no valid match found or trip taken)",
                        vehicle_id
                    );
                    state.assigned_trip_id = None;
                    state.assigned_start_date = None;
                    state.trip_confidence = 0.0;
                    
                    // Fallback to route if available
                    state.route_id = fallback_route;
                    if let Some(r) = &state.route_id {
                        println!("Vehicle {} fallback to route {}", vehicle_id, r);
                    }
                } else {
                    // Just update route if we have a guess
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

/// Find all positions where the vehicle was at or near the terminus
fn find_terminus_visits(state: &VehicleState, gtfs: &GtfsData) -> Vec<TerminusVisit> {
    let mut visits = Vec::new();

    for (idx, pos) in state.position_history.iter().enumerate() {
        let dist = haversine_distance(pos.lat, pos.lon, TERMINUS_LAT, TERMINUS_LON);

        if dist <= TERMINUS_RADIUS_METERS {
            let visit_type = classify_terminus_visit(state, gtfs, idx);
            visits.push(TerminusVisit {
                position_idx: idx,
                timestamp: pos.timestamp,
                visit_type,
            });
        }
    }

    visits
}

/// Classify whether a terminus visit is arriving (end of trip) or departing (start of trip)
fn classify_terminus_visit(
    state: &VehicleState,
    gtfs: &GtfsData,
    terminus_idx: usize,
) -> TerminusVisitType {
    let any_trip = match gtfs.trips.first() {
        Some(t) => t,
        None => return TerminusVisitType::Unknown,
    };

    let trip_stop_ids: Vec<String> = any_trip
        .stop_times
        .iter()
        .map(|st| st.stop_id.clone())
        .collect();

    // Look at positions before this terminus visit
    let before_late = state
        .position_history
        .iter()
        .take(terminus_idx)
        .rev()
        .take(5)
        .filter_map(|pos| {
            find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs).map(|(idx, _, _)| idx)
        })
        .any(|idx| idx >= LATE_STOP_THRESHOLD);

    // Look at positions after this terminus visit
    let after_early = state
        .position_history
        .iter()
        .skip(terminus_idx + 1)
        .take(5)
        .filter_map(|pos| {
            find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs).map(|(idx, _, _)| idx)
        })
        .any(|idx| idx > 0 && idx <= EARLY_STOP_THRESHOLD);

    match (before_late, after_early) {
        (true, false) => TerminusVisitType::Arriving,
        (false, true) => TerminusVisitType::Departing,
        (true, true) => TerminusVisitType::Departing, // Transition happening
        (false, false) => TerminusVisitType::Unknown,
    }
}

/// Find the segment boundary: the index in position_history after which we should score
fn find_segment_boundary(
    terminus_visits: &[TerminusVisit],
    _state: &VehicleState,
    _gtfs: &GtfsData,
) -> Option<usize> {
    // Find the most recent departing terminus visit
    terminus_visits
        .iter()
        .rev()
        .find(|v| v.visit_type == TerminusVisitType::Departing)
        .map(|v| v.position_idx)
}

/// Infer the current trip from pre-segment history.
/// Uses the pre-segment positions to find which trip the vehicle was on (T-1),
/// then returns the next trip in the block (T).
/// Also validates using terminus departure time against scheduled times.
fn infer_trip_from_pre_segment<'a>(
    state: &VehicleState,
    gtfs: &'a GtfsData,
    segment_start: usize,
    is_weekend: bool,
) -> Option<&'a Trip> {
    if segment_start == 0 {
        return None;
    }

    // Get the timestamp when we departed the terminus
    let departure_timestamp = state.position_history.get(segment_start)?.timestamp;
    let departure_la = Los_Angeles
        .timestamp_opt(departure_timestamp as i64, 0)
        .single()?;
    let departure_secs =
        (departure_la.hour() * 3600 + departure_la.minute() * 60 + departure_la.second()) as u32;

    // Score each active trip's pre-segment match
    let trip_indices = gtfs.get_active_trips(is_weekend);

    let mut best_match: Option<(&Trip, f64)> = None;

    for &idx in trip_indices {
        let trip = &gtfs.trips[idx];
        let trip_stop_ids: Vec<String> = trip
            .stop_times
            .iter()
            .map(|st| st.stop_id.clone())
            .collect();

        // Count how many pre-segment positions match late stops of this trip
        let late_stop_matches: usize = state
            .position_history
            .iter()
            .take(segment_start)
            .filter_map(|pos| {
                find_nearest_stop_on_trip(pos.lat, pos.lon, &trip_stop_ids, gtfs)
                    .map(|(idx, _, _)| idx)
            })
            .filter(|&idx| idx >= LATE_STOP_THRESHOLD)
            .count();

        if late_stop_matches >= 2 {
            // This trip is a candidate for T-1
            // Check if departure time matches the NEXT trip in this block
            if let Some(next_trip) = gtfs.get_next_trip_in_block(&trip.trip_id) {
                // Get scheduled departure time of next trip (first stop)
                if let Some(scheduled_secs) = next_trip
                    .stop_times
                    .first()
                    .and_then(|st| st.arrival_time_secs)
                {
                    // Allow 15 minute window for departure time match
                    let time_diff = (departure_secs as i32 - scheduled_secs as i32).abs();
                    if time_diff <= 900 {
                        let score = late_stop_matches as f64 + (1.0 - time_diff as f64 / 900.0);
                        if best_match.as_ref().map_or(true, |(_, s)| score > *s) {
                            best_match = Some((next_trip, score));
                        }
                    }
                }
            }
        }
    }

    best_match.map(|(trip, _)| trip)
}

/// Score a trip using segmented history
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

    let start_idx = segment_start.unwrap_or(0);
    let mut matches: Vec<StopMatch> = Vec::new();

    for pos in state.position_history.iter().skip(start_idx) {
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
        return (0.0, false);
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

                // typically vehicles run more late than early but a few min early is also acceptable to them
                // Adjusted to be more symmetrical relative to zero, but biased towards lateness
                let ts = if delta >= -300 && delta <= 600 {
                    // Up to 5 mins early, or 10 mins late
                    1.0
                } else if delta >= -600 && delta <= 1200 {
                    // Up to 10 mins early, or 20 mins late
                    0.7
                } else if delta >= -1200 && delta <= 1800 {
                    // Up to 20 mins early, or 30 mins late
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

    // Check for transition signature
    let transition_detected = detect_transition_signature(state, trip, gtfs, terminus_visits);

    // Consider early stop presence in recent history
    let has_early_stops = matches
        .iter()
        .any(|m| m.stop_idx > 0 && m.stop_idx <= EARLY_STOP_THRESHOLD);
    let early_bonus = if has_early_stops && segment_start.is_some() {
        1.2
    } else {
        1.0
    };

    let raw_score = (unique_stops as f64).sqrt()
        * order_score
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
        .any(|idx| idx >= LATE_STOP_THRESHOLD);

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

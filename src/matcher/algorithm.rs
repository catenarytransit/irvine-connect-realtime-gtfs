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
            println!("Vehicle {} visiting stop {} at {}", state.vehicle_id, stop_id, timestamp);
        }
    }
}

pub fn perform_global_assignment(state_manager: &mut crate::matcher::VehicleStateManager, gtfs: &GtfsData) {
    let mut all_matches = Vec::new(); // (score, vehicle_id, trip_id)
    
    // We need to know which vehicles are active to iterate over them
    // state_manager.all_states() returns an iterator
    // We need mutable access later to update them, so we might need a different pattern 
    // or collect IDs first.
    
    // Since we need to update states later, let's collect IDs first
    let vehicle_ids: Vec<String> = state_manager.all_states().map(|s| s.vehicle_id.clone()).collect();
    
    // 1. Calculate scores for all vehicles and all candidates
    for vehicle_id in &vehicle_ids {
        if let Some(state) = state_manager.get(vehicle_id) {
            // Skip assignment if not enough stops visited
            if state.stop_visit_timestamps.len() < MIN_STOPS_FOR_ASSIGNMENT {
                continue;
            }

            let last_timestamp = state.last_stop_visit_time;
            let la_time = Los_Angeles.timestamp_opt(last_timestamp as i64, 0).single();
            
            if let Some(now) = la_time {
                let is_weekend = now.weekday().num_days_from_monday() >= 5;
                let current_secs = (now.hour() * 3600 + now.minute() * 60 + now.second()) as u32;

                let candidates = find_candidate_trips(gtfs, is_weekend, current_secs);
                println!("Found {} candidates for vehicle {}", candidates.len(), vehicle_id);

                for trip in candidates {
                    let score = score_trip_match(&state.stop_visit_timestamps, trip, current_secs);
                    if score > 0.0 {
                         println!("Vehicle {} - Trip {} score: {:.3}", vehicle_id, trip.trip_id, score);
                        // Add hysteresis: if this is the currently assigned trip, boost the score slightly
                        let final_score = if state.assigned_trip_id.as_deref() == Some(&trip.trip_id) {
                            score + 0.1 // Hysteresis bonus
                        } else {
                            score
                        };
                        
                        // Store match: (score, vehicle_id, trip_id, start_date)
                        all_matches.push((final_score, vehicle_id.clone(), trip.trip_id.clone(), now.format("%Y%m%d").to_string()));
                    }
                }
            }
        }
    }

    // 2. Sort matches by score descending
    // Note: f64 doesn't implement Ord, so we use partial_cmp
    all_matches.sort_by(|a, b| b.0.partial_cmp(&a.0).unwrap_or(std::cmp::Ordering::Equal));

    // 3. Assign trips greedily respecting uniqueness
    let mut assigned_trips = std::collections::HashSet::new();
    let mut assigned_vehicles = std::collections::HashSet::new();
    
    // We need to apply assignments. Since we can't mutate state_manager while iterating over matches (if we hold refs),
    // we'll first determine the best assignment for each vehicle.
    // Actually, we can just iterate the sorted matches and apply if both are free.
    
    let mut final_assignments = std::collections::HashMap::new();

    for (score, vehicle_id, trip_id, start_date) in all_matches {
        if score < CONFIDENCE_THRESHOLD {
            continue;
        }

        if !assigned_vehicles.contains(&vehicle_id) && !assigned_trips.contains(&trip_id) {
            // Assign!
            assigned_vehicles.insert(vehicle_id.clone());
            assigned_trips.insert(trip_id.clone());
            final_assignments.insert(vehicle_id, (trip_id, start_date, score));
        }
    }

    // 4. Update the actual states
    // First, clear assignments for vehicles that didn't get a match (or maybe we should keep old if decent? No, global assignment implies re-eval)
    // Actually, "clearing" might cause flickering if we just had a temporary drop in score. 
    // But uniqueness requires us to be strict.
    // Let's iterate all vehicles.
    
    for vehicle_id in vehicle_ids {
        if let Some(state) = state_manager.get_mut(&vehicle_id) {
             // Handle trip transition logic here if needed? 
             // The original code had `should_transition_trip`. 
             // That logic checks if we are at the end of a trip. 
             // Global assignment should ideally handle this if the new trip scores higher.
             // But valid "next trip" might not have started yet (score low).
             // Let's keep the transition logic but integrating it is tricky.
             // For now, let's just apply the best matches found.
             
             if let Some((trip_id, start_date, score)) = final_assignments.get(&vehicle_id) {
                 if state.assigned_trip_id.as_ref() != Some(trip_id) {
                     println!("Assigning trip {} to vehicle {} (score: {:.3})", trip_id, vehicle_id, score);
                     state.assigned_trip_id = Some(trip_id.clone());
                     state.assigned_start_date = Some(start_date.clone());
                     state.trip_confidence = *score;
                 } else {
                     // Update confidence
                     state.trip_confidence = *score;
                 }
             } else {
                 // No match found this round. 
                 // Should we clear? If we don't clear, we might hold onto a stale trip that another bus needs.
                 // If we DO clear, we might flicker.
                 // Given the uniqueness constraint, if we didn't get the trip in the global assignment, 
                 // it means either score was too low OR someone else took it.
                 // In either case, we should probably clear or at least acknowledge we failed to match.
                if state.assigned_trip_id.is_some() {
                     println!("Vehicle {} lost its assignment (no valid match found or trip taken)", vehicle_id);
                     state.assigned_trip_id = None;
                     state.assigned_start_date = None;
                     state.trip_confidence = 0.0;
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


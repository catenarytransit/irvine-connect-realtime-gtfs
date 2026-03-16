use crate::gtfs::{GtfsData, Trip};
use crate::matcher::history::TimestampedPosition;
use crate::matcher::proximity::{emission_probability, haversine_distance, transition_probability};
use chrono::TimeZone;
use chrono_tz::America::Los_Angeles;

const GPS_SIGMA_Z: f64 = 25.0;
const TRANSITION_BETA: f64 = 200.0;
const STATE_WINDOW: usize = 15;
const MIN_EMISSION_LOG: f64 = -50.0;
// Temporal: tolerance window (seconds) before penalizing schedule deviation
const TEMPORAL_TOLERANCE_SECS: f64 = 900.0;
// Temporal: rate of decay beyond the tolerance window
const TEMPORAL_SIGMA: f64 = 1200.0;

pub struct ViterbiResult {
    pub score: f64,
    pub matched_stops: Vec<Option<u64>>,
}

fn build_cumulative_distances(trip: &Trip, gtfs: &GtfsData) -> Vec<f64> {
    let mut cumulative = vec![0.0; trip.stop_times.len()];

    for i in 1..trip.stop_times.len() {
        let prev_id = &trip.stop_times[i - 1].stop_id;
        let curr_id = &trip.stop_times[i].stop_id;

        let seg_dist = match (gtfs.stops.get(prev_id), gtfs.stops.get(curr_id)) {
            (Some(a), Some(b)) => haversine_distance(a.lat, a.lon, b.lat, b.lon),
            _ => 0.0,
        };

        cumulative[i] = cumulative[i - 1] + seg_dist;
    }

    cumulative
}

fn route_distance(cumulative: &[f64], from: usize, to: usize) -> f64 {
    if to >= from {
        cumulative[to] - cumulative[from]
    } else {
        // Backward transitions: return a distance so large that
        // |d_route - d_euclidean| produces a massive penalty in the
        // exponential transition probability.
        1.0e6
    }
}

/// Compute the base timestamp for a service date (midnight LA time).
fn service_day_base(date_str: &str) -> Option<i64> {
    use chrono::NaiveDate;
    let date = NaiveDate::parse_from_str(date_str, "%Y%m%d").ok()?;
    let naive_dt = date.and_hms_opt(0, 0, 0)?;
    let base_dt = Los_Angeles.from_local_datetime(&naive_dt).single()?;
    Some(base_dt.timestamp())
}

/// Temporal emission penalty: penalizes a stop match when the GPS timestamp
/// is far from the stop's scheduled time. Returns a log-space penalty.
fn temporal_log_penalty(gps_ts: u64, sched_secs: Option<u32>, base_ts: Option<i64>) -> f64 {
    let (sched, base) = match (sched_secs, base_ts) {
        (Some(s), Some(b)) => (s, b),
        _ => return 0.0, // no schedule info — no penalty
    };

    let sched_ts = base + sched as i64;
    let delta = (gps_ts as i64 - sched_ts).abs() as f64;

    if delta <= TEMPORAL_TOLERANCE_SECS {
        0.0
    } else {
        let excess = delta - TEMPORAL_TOLERANCE_SECS;
        -(excess * excess) / (2.0 * TEMPORAL_SIGMA * TEMPORAL_SIGMA)
    }
}

pub fn viterbi_score(
    positions: &[&TimestampedPosition],
    trip: &Trip,
    gtfs: &GtfsData,
    date_str: &str,
) -> ViterbiResult {
    let empty = ViterbiResult {
        score: 0.0,
        matched_stops: vec![None; trip.stop_times.len()],
    };

    let num_stops = trip.stop_times.len();
    if positions.is_empty() || num_stops == 0 {
        return empty;
    }

    let cumulative = build_cumulative_distances(trip, gtfs);
    let base_ts = service_day_base(date_str);

    let stop_coords: Vec<Option<(f64, f64)>> = trip
        .stop_times
        .iter()
        .map(|st| gtfs.stops.get(&st.stop_id).map(|s| (s.lat, s.lon)))
        .collect();

    let mut prev_log_prob: Vec<f64> = vec![f64::NEG_INFINITY; num_stops];
    let mut best_path: Vec<Vec<usize>> = vec![Vec::new(); num_stops];
    let mut window_center: usize = 0;

    // === Initialization ===
    let first_pos = positions[0];
    let mut init_best_j = 0;
    let mut init_best_log = f64::NEG_INFINITY;

    let lo = 0;
    let hi = num_stops.min(STATE_WINDOW * 2);

    for j in lo..hi {
        if let Some((slat, slon)) = stop_coords[j] {
            let dist = haversine_distance(first_pos.lat, first_pos.lon, slat, slon);
            let ep = emission_probability(dist, GPS_SIGMA_Z);
            let log_ep = if ep > 0.0 {
                ep.ln().max(MIN_EMISSION_LOG)
            } else {
                MIN_EMISSION_LOG
            };

            let time_pen = temporal_log_penalty(
                first_pos.timestamp,
                trip.stop_times[j].arrival_time_secs,
                base_ts,
            );

            let total = log_ep + time_pen;
            prev_log_prob[j] = total;
            best_path[j] = vec![j];

            if total > init_best_log {
                init_best_log = total;
                init_best_j = j;
            }
        }
    }

    window_center = init_best_j;

    // === Recursion ===
    for t in 1..positions.len() {
        let pos = positions[t];
        let prev_pos = positions[t - 1];
        let d_euclidean = haversine_distance(prev_pos.lat, prev_pos.lon, pos.lat, pos.lon);

        let gps_dlat = pos.lat - prev_pos.lat;
        let gps_dlon = pos.lon - prev_pos.lon;

        let mut curr_log_prob: Vec<f64> = vec![f64::NEG_INFINITY; num_stops];
        let mut curr_path: Vec<Vec<usize>> = vec![Vec::new(); num_stops];
        let mut step_best_j = window_center;
        let mut step_best_log = f64::NEG_INFINITY;

        let j_lo = window_center.saturating_sub(STATE_WINDOW);
        let j_hi = (window_center + STATE_WINDOW + 1).min(num_stops);

        for j in j_lo..j_hi {
            let (slat, slon) = match stop_coords[j] {
                Some(c) => c,
                None => continue,
            };

            let dist = haversine_distance(pos.lat, pos.lon, slat, slon);
            let ep = emission_probability(dist, GPS_SIGMA_Z);
            let log_ep = if ep > 0.0 {
                ep.ln().max(MIN_EMISSION_LOG)
            } else {
                MIN_EMISSION_LOG
            };

            let time_pen = temporal_log_penalty(
                pos.timestamp,
                trip.stop_times[j].arrival_time_secs,
                base_ts,
            );

            let i_lo = j_lo;
            let i_hi = (j + 1).min(j_hi);

            let mut best_prev_log = f64::NEG_INFINITY;
            let mut best_prev_i = j;

            for i in i_lo..i_hi {
                if prev_log_prob[i] == f64::NEG_INFINITY {
                    continue;
                }

                let d_route = route_distance(&cumulative, i, j);
                let tp = transition_probability(d_route, d_euclidean, TRANSITION_BETA);
                let log_tp = if tp > 0.0 {
                    tp.ln().max(MIN_EMISSION_LOG)
                } else {
                    MIN_EMISSION_LOG
                };

                // Velocity vector consistency
                let dir_log = if i != j {
                    if let (Some((i_lat, i_lon)), Some((j_lat, j_lon))) =
                        (stop_coords[i], stop_coords[j])
                    {
                        let route_dlat = j_lat - i_lat;
                        let route_dlon = j_lon - i_lon;
                        let dot = gps_dlat * route_dlat + gps_dlon * route_dlon;
                        let gps_mag = (gps_dlat * gps_dlat + gps_dlon * gps_dlon).sqrt();
                        let route_mag =
                            (route_dlat * route_dlat + route_dlon * route_dlon).sqrt();

                        if gps_mag > 1e-9 && route_mag > 1e-9 {
                            let cos_angle = (dot / (gps_mag * route_mag)).clamp(-1.0, 1.0);
                            let factor = ((1.0 + cos_angle) / 2.0).powi(2);
                            factor.max(0.01).ln()
                        } else {
                            0.0
                        }
                    } else {
                        0.0
                    }
                } else {
                    0.0
                };

                let candidate = prev_log_prob[i] + log_tp + dir_log;
                if candidate > best_prev_log {
                    best_prev_log = candidate;
                    best_prev_i = i;
                }
            }

            let total = best_prev_log + log_ep + time_pen;
            curr_log_prob[j] = total;

            let mut path = best_path[best_prev_i].clone();
            path.push(j);
            curr_path[j] = path;

            if total > step_best_log {
                step_best_log = total;
                step_best_j = j;
            }
        }

        prev_log_prob = curr_log_prob;
        best_path = curr_path;
        window_center = step_best_j;
    }

    // === Termination ===
    let mut best_final_j = 0;
    let mut best_final_log = f64::NEG_INFINITY;

    for j in 0..num_stops {
        if prev_log_prob[j] > best_final_log {
            best_final_log = prev_log_prob[j];
            best_final_j = j;
        }
    }

    if best_final_log == f64::NEG_INFINITY {
        return empty;
    }

    // === Traceback ===
    let winning_path = &best_path[best_final_j];

    let mut matched_stops: Vec<Option<u64>> = vec![None; num_stops];

    for (obs_idx, &stop_idx) in winning_path.iter().enumerate() {
        if obs_idx < positions.len() && matched_stops[stop_idx].is_none() {
            matched_stops[stop_idx] = Some(positions[obs_idx].timestamp);
        }
    }

    let unique_matched = matched_stops.iter().filter(|m| m.is_some()).count();
    if unique_matched == 0 {
        return empty;
    }

    let coverage = unique_matched as f64 / num_stops as f64;
    let avg_log_prob = best_final_log / positions.len() as f64;
    let confidence = (avg_log_prob / MIN_EMISSION_LOG.abs())
        .exp()
        .clamp(0.0, 1.0);

    let raw_score = (unique_matched as f64).sqrt() * (0.3 + 0.7 * coverage) * confidence;

    ViterbiResult {
        score: raw_score.min(1.0),
        matched_stops,
    }
}

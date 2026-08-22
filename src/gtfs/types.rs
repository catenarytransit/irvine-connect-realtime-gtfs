use std::collections::HashMap;

const EARTH_RADIUS_M: f64 = 6_371_000.0;

#[derive(Debug, Clone)]
pub struct Stop {
    pub id: String,
    pub name: String,
    pub lat: f64,
    pub lon: f64,
}

#[derive(Debug, Clone)]
pub struct StopTime {
    pub stop_id: String,
    pub sequence: u32,
    pub arrival_time: String,
    pub arrival_time_secs: Option<u32>,
    pub departure_time: String,
    pub departure_time_secs: Option<u32>,
}

#[derive(Debug, Clone)]
pub struct Trip {
    pub trip_id: String,
    pub route_id: String,
    pub service_id: String,
    pub block_id: String,
    pub stop_times: Vec<StopTime>,
    pub start_time_minutes: Option<u32>,
    pub stop_coords: Vec<Option<(f64, f64)>>,
    pub cumulative_distances: Vec<f64>,
}

impl Trip {
    pub fn new(
        trip_id: String,
        route_id: String,
        service_id: String,
        block_id: String,
        stop_times: Vec<StopTime>,
        stops: &HashMap<String, Stop>,
    ) -> Self {
        let start_time_minutes = stop_times
            .first()
            .and_then(|st| st.arrival_time_secs)
            .map(|secs| secs / 60);

        let stop_coords: Vec<Option<(f64, f64)>> = stop_times
            .iter()
            .map(|st| stops.get(&st.stop_id).map(|stop| (stop.lat, stop.lon)))
            .collect();

        let mut cumulative_distances = vec![0.0; stop_times.len()];
        for i in 1..stop_times.len() {
            let segment_distance = match (stop_coords[i - 1], stop_coords[i]) {
                (Some((a_lat, a_lon)), Some((b_lat, b_lon))) => {
                    haversine_distance(a_lat, a_lon, b_lat, b_lon)
                }
                _ => 0.0,
            };
            cumulative_distances[i] = cumulative_distances[i - 1] + segment_distance;
        }

        Self {
            trip_id,
            route_id,
            service_id,
            block_id,
            stop_times,
            start_time_minutes,
            stop_coords,
            cumulative_distances,
        }
    }

    pub fn first_stop_id(&self) -> Option<&str> {
        self.stop_times.first().map(|st| st.stop_id.as_str())
    }

    pub fn last_stop_id(&self) -> Option<&str> {
        self.stop_times.last().map(|st| st.stop_id.as_str())
    }

    pub fn start_time_minutes(&self) -> Option<u32> {
        self.start_time_minutes
    }
}

fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let delta_lat = (lat2 - lat1).to_radians();
    let delta_lon = (lon2 - lon1).to_radians();

    let a = (delta_lat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();

    EARTH_RADIUS_M * c
}

#[derive(Debug)]
pub struct GtfsData {
    pub stops: HashMap<String, Stop>,
    pub trips: Vec<Trip>,
    pub trips_by_block: HashMap<String, Vec<usize>>,
    pub weekday_trips: Vec<usize>,
    pub weekend_trips: Vec<usize>,
    trip_index_by_id: HashMap<String, usize>,
    previous_trip_index: Vec<Option<usize>>,
    next_trip_index: Vec<Option<usize>>,
}

impl GtfsData {
    pub fn new(
        stops: HashMap<String, Stop>,
        trips: Vec<Trip>,
        trips_by_block: HashMap<String, Vec<usize>>,
        weekday_trips: Vec<usize>,
        weekend_trips: Vec<usize>,
    ) -> Self {
        let trip_index_by_id = trips
            .iter()
            .enumerate()
            .map(|(idx, trip)| (trip.trip_id.clone(), idx))
            .collect();

        let mut previous_trip_index = vec![None; trips.len()];
        let mut next_trip_index = vec![None; trips.len()];
        for block_indices in trips_by_block.values() {
            for pair in block_indices.windows(2) {
                previous_trip_index[pair[1]] = Some(pair[0]);
                next_trip_index[pair[0]] = Some(pair[1]);
            }
        }

        Self {
            stops,
            trips,
            trips_by_block,
            weekday_trips,
            weekend_trips,
            trip_index_by_id,
            previous_trip_index,
            next_trip_index,
        }
    }

    pub fn get_active_trips(&self, is_weekend: bool) -> &[usize] {
        if is_weekend {
            &self.weekend_trips
        } else {
            &self.weekday_trips
        }
    }

    pub fn get_trips_in_block(&self, block_id: &str) -> Vec<&Trip> {
        self.trips_by_block
            .get(block_id)
            .map(|indices| indices.iter().map(|&i| &self.trips[i]).collect())
            .unwrap_or_default()
    }

    pub fn get_trip_by_id(&self, trip_id: &str) -> Option<&Trip> {
        self.trip_index_by_id
            .get(trip_id)
            .map(|&idx| &self.trips[idx])
    }

    /// Get the next trip in the same block, ordered by start time.
    /// Returns None if this is the last trip in the block or trip not found.
    pub fn get_next_trip_in_block(&self, current_trip_id: &str) -> Option<&Trip> {
        let current_idx = *self.trip_index_by_id.get(current_trip_id)?;
        self.next_trip_index[current_idx].map(|idx| &self.trips[idx])
    }

    /// Get the previous trip in the same block, ordered by start time.
    /// Returns None if this is the first trip in the block or trip not found.
    pub fn get_previous_trip_in_block(&self, current_trip_id: &str) -> Option<&Trip> {
        let current_idx = *self.trip_index_by_id.get(current_trip_id)?;
        self.previous_trip_index[current_idx].map(|idx| &self.trips[idx])
    }
}

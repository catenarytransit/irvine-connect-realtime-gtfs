use std::collections::HashMap;

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
}

impl Trip {
    pub fn first_stop_id(&self) -> Option<&str> {
        self.stop_times.first().map(|st| st.stop_id.as_str())
    }

    pub fn last_stop_id(&self) -> Option<&str> {
        self.stop_times.last().map(|st| st.stop_id.as_str())
    }

    pub fn start_time_minutes(&self) -> Option<u32> {
        self.stop_times
            .first()
            .and_then(|st| parse_time_to_minutes(&st.arrival_time))
    }
}

fn parse_time_to_minutes(time_str: &str) -> Option<u32> {
    let parts: Vec<&str> = time_str.split(':').collect();
    if parts.len() >= 2 {
        let hours: u32 = parts[0].parse().ok()?;
        let mins: u32 = parts[1].parse().ok()?;
        Some(hours * 60 + mins)
    } else {
        None
    }
}

#[derive(Debug)]
pub struct GtfsData {
    pub stops: HashMap<String, Stop>,
    pub trips: Vec<Trip>,
    pub trips_by_block: HashMap<String, Vec<usize>>,
    pub weekday_trips: Vec<usize>,
    pub weekend_trips: Vec<usize>,
}

impl GtfsData {
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
        self.trips.iter().find(|t| t.trip_id == trip_id)
    }

    /// Get the next trip in the same block, ordered by start time.
    /// Returns None if this is the last trip in the block or trip not found.
    pub fn get_next_trip_in_block(&self, current_trip_id: &str) -> Option<&Trip> {
        let current_trip = self.get_trip_by_id(current_trip_id)?;
        let block_indices = self.trips_by_block.get(&current_trip.block_id)?;

        let current_idx = block_indices
            .iter()
            .position(|&i| self.trips[i].trip_id == current_trip_id)?;

        if current_idx + 1 < block_indices.len() {
            Some(&self.trips[block_indices[current_idx + 1]])
        } else {
            None
        }
    }

    /// Get the previous trip in the same block, ordered by start time.
    /// Returns None if this is the first trip in the block or trip not found.
    pub fn get_previous_trip_in_block(&self, current_trip_id: &str) -> Option<&Trip> {
        let current_trip = self.get_trip_by_id(current_trip_id)?;
        let block_indices = self.trips_by_block.get(&current_trip.block_id)?;

        let current_idx = block_indices
            .iter()
            .position(|&i| self.trips[i].trip_id == current_trip_id)?;

        if current_idx > 0 {
            Some(&self.trips[block_indices[current_idx - 1]])
        } else {
            None
        }
    }
}

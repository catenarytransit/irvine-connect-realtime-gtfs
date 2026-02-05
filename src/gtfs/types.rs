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
}

#[derive(Debug, Clone)]
pub struct Trip {
    pub trip_id: String,
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
}

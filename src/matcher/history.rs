use std::collections::{HashMap, VecDeque};

const HISTORY_DURATION_SECS: u64 = 20 * 60;
const MAX_POSITION_HISTORY: usize = 1200;

#[derive(Debug, Clone)]
pub struct TimestampedPosition {
    pub lat: f64,
    pub lon: f64,
    pub bearing: Option<f32>,
    pub timestamp: u64,
}

#[derive(Debug, Clone)]
pub struct VehicleState {
    pub vehicle_id: String,
    pub position_history: VecDeque<TimestampedPosition>,
    pub visited_stops: Vec<String>,
    pub stop_visit_timestamps: Vec<(String, u64)>,
    pub assigned_trip_id: Option<String>,
    pub assigned_start_date: Option<String>,
    pub trip_confidence: f64,
    pub last_stop_visit_time: u64,
}

impl VehicleState {
    pub fn new(vehicle_id: String) -> Self {
        Self {
            vehicle_id,
            position_history: VecDeque::with_capacity(MAX_POSITION_HISTORY),
            visited_stops: Vec::new(),
            stop_visit_timestamps: Vec::new(),
            assigned_trip_id: None,
            assigned_start_date: None,
            trip_confidence: 0.0,
            last_stop_visit_time: 0,
        }
    }

    pub fn add_position(&mut self, lat: f64, lon: f64, bearing: Option<f32>, timestamp: u64) {
        self.position_history.push_back(TimestampedPosition {
            lat,
            lon,
            bearing,
            timestamp,
        });

        if self.position_history.len() > MAX_POSITION_HISTORY {
            self.position_history.pop_front();
        }

        let cutoff = timestamp.saturating_sub(HISTORY_DURATION_SECS);
        while self
            .position_history
            .front()
            .map(|p| p.timestamp < cutoff)
            .unwrap_or(false)
        {
            self.position_history.pop_front();
        }
    }

    pub fn record_stop_visit(&mut self, stop_id: &str, timestamp: u64) {
        if self
            .visited_stops
            .last()
            .map(|s| s != stop_id)
            .unwrap_or(true)
        {
            self.visited_stops.push(stop_id.to_string());
            self.stop_visit_timestamps
                .push((stop_id.to_string(), timestamp));
            self.last_stop_visit_time = timestamp;
        }
    }

    pub fn clear_for_new_trip(&mut self) {
        self.visited_stops.clear();
        self.stop_visit_timestamps.clear();
        self.assigned_trip_id = None;
        self.assigned_start_date = None;
        self.trip_confidence = 0.0;
    }
}

#[derive(Debug, Default)]
pub struct VehicleStateManager {
    states: HashMap<String, VehicleState>,
}

impl VehicleStateManager {
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
        }
    }

    pub fn get_or_create(&mut self, vehicle_id: &str) -> &mut VehicleState {
        self.states
            .entry(vehicle_id.to_string())
            .or_insert_with(|| VehicleState::new(vehicle_id.to_string()))
    }

    pub fn get(&self, vehicle_id: &str) -> Option<&VehicleState> {
        self.states.get(vehicle_id)
    }

    pub fn all_states(&self) -> impl Iterator<Item = &VehicleState> {
        self.states.values()
    }
}

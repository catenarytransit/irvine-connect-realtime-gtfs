use serde::{Deserialize, Serialize};
use std::collections::{HashMap, VecDeque};
use std::fs::File;
use std::io::{BufReader, BufWriter};

const HISTORY_DURATION_SECS: u64 = 20 * 60;
const MAX_POSITION_HISTORY: usize = 1200;

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct TimestampedPosition {
    pub lat: f64,
    pub lon: f64,
    pub bearing: Option<f32>,
    pub timestamp: u64,
}

#[derive(Debug, Clone, Serialize, Deserialize)]
pub struct VehicleState {
    pub vehicle_id: String,
    pub label: Option<String>,
    pub source_id: Option<String>,
    pub position_history: VecDeque<TimestampedPosition>,
    pub visited_stops: Vec<String>,
    pub stop_visit_timestamps: Vec<(String, u64)>,
    pub assigned_trip_id: Option<String>,
    pub route_id: Option<String>,
    pub assigned_start_date: Option<String>,
    pub trip_confidence: f64,
    pub last_stop_visit_time: u64,
    /// Trip ID of the previous trip in the block, for transition detection
    pub previous_trip_id: Option<String>,
    /// Timestamp when we last detected departure from terminus
    pub last_terminus_departure: Option<u64>,
}

impl VehicleState {
    pub fn new(vehicle_id: String) -> Self {
        Self {
            vehicle_id,
            label: None,
            source_id: None,
            position_history: VecDeque::with_capacity(MAX_POSITION_HISTORY),
            visited_stops: Vec::new(),
            stop_visit_timestamps: Vec::new(),
            assigned_trip_id: None,
            route_id: None,
            assigned_start_date: None,
            trip_confidence: 0.0,
            last_stop_visit_time: 0,
            previous_trip_id: None,
            last_terminus_departure: None,
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
        self.route_id = None;
        self.trip_confidence = 0.0;
    }

    /// Called when we detect the vehicle has departed terminus on a new trip.
    /// Preserves position history but marks the transition point.
    pub fn transition_to_new_trip(&mut self, new_trip_id: &str, departure_timestamp: u64) {
        self.previous_trip_id = self.assigned_trip_id.take();
        self.assigned_trip_id = Some(new_trip_id.to_string());
        self.last_terminus_departure = Some(departure_timestamp);
        self.visited_stops.clear();
        self.stop_visit_timestamps.clear();
        // route_id will be updated by the next assignment loop if needed,
        // but ideally we should set it here if we knew it.
        // For now, the global assignment loop will handle setting route_id.
    }

    /// Clear position history entirely (use when transitioning between blocks or long gaps)
    pub fn clear_position_history(&mut self) {
        self.position_history.clear();
    }
}

#[derive(Debug, Default, Serialize, Deserialize)]
pub struct VehicleStateManager {
    states: HashMap<String, VehicleState>,
}

impl VehicleStateManager {
    pub fn new() -> Self {
        Self {
            states: HashMap::new(),
        }
    }

    pub fn load(path: &str) -> std::io::Result<Self> {
        let file = File::open(path)?;
        let reader = BufReader::new(file);
        let states = serde_json::from_reader(reader)?;
        Ok(states)
    }

    pub fn save(&self, path: &str) -> std::io::Result<()> {
        let file = File::create(path)?;
        let writer = BufWriter::new(file);
        serde_json::to_writer(writer, self)?;
        Ok(())
    }

    pub fn get_or_create(&mut self, vehicle_id: &str) -> &mut VehicleState {
        self.states
            .entry(vehicle_id.to_string())
            .or_insert_with(|| VehicleState::new(vehicle_id.to_string()))
    }

    pub fn get(&self, vehicle_id: &str) -> Option<&VehicleState> {
        self.states.get(vehicle_id)
    }

    pub fn get_mut(&mut self, vehicle_id: &str) -> Option<&mut VehicleState> {
        self.states.get_mut(vehicle_id)
    }

    pub fn all_states(&self) -> impl Iterator<Item = &VehicleState> {
        self.states.values()
    }
}

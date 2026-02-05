use crate::gtfs::{GtfsData, Stop};

const EARTH_RADIUS_M: f64 = 6_371_000.0;
const STOP_RADIUS_METERS: f64 = 50.0;

pub fn haversine_distance(lat1: f64, lon1: f64, lat2: f64, lon2: f64) -> f64 {
    let lat1_rad = lat1.to_radians();
    let lat2_rad = lat2.to_radians();
    let delta_lat = (lat2 - lat1).to_radians();
    let delta_lon = (lon2 - lon1).to_radians();

    let a = (delta_lat / 2.0).sin().powi(2)
        + lat1_rad.cos() * lat2_rad.cos() * (delta_lon / 2.0).sin().powi(2);
    let c = 2.0 * a.sqrt().asin();

    EARTH_RADIUS_M * c
}

pub fn find_nearby_stops(lat: f64, lon: f64, gtfs: &GtfsData) -> Vec<(String, f64)> {
    let mut nearby = Vec::new();
    
    for stop in gtfs.stops.values() {
        let dist = haversine_distance(lat, lon, stop.lat, stop.lon);
        if dist <= STOP_RADIUS_METERS {
            nearby.push((stop.id.clone(), dist));
        }
    }
    
    nearby.sort_by(|a, b| a.1.partial_cmp(&b.1).unwrap());
    nearby
}

pub fn find_nearest_stop(lat: f64, lon: f64, gtfs: &GtfsData) -> Option<(String, f64)> {
    find_nearby_stops(lat, lon, gtfs).into_iter().next()
}

pub fn is_stop_directional(stop: &Stop) -> Option<bool> {
    let name = &stop.name;
    if name.ends_with(" IB") || name.contains(" IB ") {
        Some(true)
    } else if name.ends_with(" OB") || name.contains(" OB ") {
        Some(false)
    } else {
        None
    }
}

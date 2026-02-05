use crate::gtfs::GtfsData;

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

pub fn find_nearest_stop_on_trip(
    lat: f64,
    lon: f64,
    trip_stop_ids: &[String],
    gtfs: &GtfsData,
) -> Option<(usize, String, f64)> {
    let mut best: Option<(usize, String, f64)> = None;

    for (idx, stop_id) in trip_stop_ids.iter().enumerate() {
        if let Some(stop) = gtfs.stops.get(stop_id) {
            let dist = haversine_distance(lat, lon, stop.lat, stop.lon);
            if dist <= STOP_RADIUS_METERS {
                if best.as_ref().map_or(true, |(_, _, d)| dist < *d) {
                    best = Some((idx, stop_id.clone(), dist));
                }
            }
        }
    }

    best
}

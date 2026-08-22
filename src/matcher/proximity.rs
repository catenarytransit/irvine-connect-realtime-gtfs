use crate::gtfs::Trip;

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

pub fn find_nearest_stop_on_trip(lat: f64, lon: f64, trip: &Trip) -> Option<(usize, f64)> {
    let mut best: Option<(usize, f64)> = None;

    for (idx, coords) in trip.stop_coords.iter().enumerate() {
        if let Some((stop_lat, stop_lon)) = coords {
            let dist = haversine_distance(lat, lon, *stop_lat, *stop_lon);
            if dist <= STOP_RADIUS_METERS && best.as_ref().map_or(true, |(_, d)| dist < *d) {
                best = Some((idx, dist));
            }
        }
    }

    best
}

/// Perpendicular distance from a point to the great-circle segment A→B,
/// clamped to the segment endpoints. Returns (distance_meters, along_track_fraction)
/// where fraction 0.0 = at A, 1.0 = at B.
pub fn cross_track_distance(
    lat: f64,
    lon: f64,
    a_lat: f64,
    a_lon: f64,
    b_lat: f64,
    b_lon: f64,
) -> (f64, f64) {
    let seg_len = haversine_distance(a_lat, a_lon, b_lat, b_lon);
    if seg_len < 1.0 {
        let d = haversine_distance(lat, lon, a_lat, a_lon);
        return (d, 0.0);
    }

    let d_ap = haversine_distance(a_lat, a_lon, lat, lon);
    let d_bp = haversine_distance(b_lat, b_lon, lat, lon);

    // Along-track fraction via projection
    // t = (dAP² + dAB² - dBP²) / (2 * dAP * dAB)  ... but simpler:
    // t = (dAP² - dBP² + dAB²) / (2 * dAB²)
    let t = (d_ap * d_ap - d_bp * d_bp + seg_len * seg_len) / (2.0 * seg_len * seg_len);
    let t_clamped = t.clamp(0.0, 1.0);

    if t_clamped <= 0.0 {
        return (d_ap, 0.0);
    }
    if t_clamped >= 1.0 {
        return (d_bp, 1.0);
    }

    // Interpolate the projected point on the segment
    let a_lat_r = a_lat.to_radians();
    let a_lon_r = a_lon.to_radians();
    let b_lat_r = b_lat.to_radians();
    let b_lon_r = b_lon.to_radians();

    let proj_lat = a_lat_r + t_clamped * (b_lat_r - a_lat_r);
    let proj_lon = a_lon_r + t_clamped * (b_lon_r - a_lon_r);

    let dist = haversine_distance(lat, lon, proj_lat.to_degrees(), proj_lon.to_degrees());
    (dist, t_clamped)
}

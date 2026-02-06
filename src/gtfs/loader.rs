use crate::gtfs::{GtfsData, Stop, StopTime, Trip};
use std::collections::HashMap;
use std::io::{Cursor, Read};

const GTFS_URL: &str = "https://github.com/catenarytransit/irvine-connect-schedule-gtfs/releases/download/latest/gtfs.zip";

pub async fn load_gtfs() -> Result<GtfsData, Box<dyn std::error::Error + Send + Sync>> {
    println!("Downloading GTFS from {}", GTFS_URL);

    let response = reqwest::get(GTFS_URL).await?;
    let bytes = response.bytes().await?;

    println!("Downloaded {} bytes, extracting...", bytes.len());

    let cursor = Cursor::new(bytes.as_ref());
    let mut archive = zip::ZipArchive::new(cursor)?;

    let stops = parse_stops(&mut archive)?;
    let trips_raw = parse_trips(&mut archive)?;
    let stop_times_map = parse_stop_times(&mut archive)?;

    let mut trips: Vec<Trip> = Vec::new();
    let mut trips_by_block: HashMap<String, Vec<usize>> = HashMap::new();
    let mut weekday_trips: Vec<usize> = Vec::new();
    let mut weekend_trips: Vec<usize> = Vec::new();

    for (route_id, trip_id, service_id, block_id) in trips_raw {
        let stop_times = stop_times_map.get(&trip_id).cloned().unwrap_or_default();

        let idx = trips.len();

        if service_id == "Weekday" {
            weekday_trips.push(idx);
        } else {
            weekend_trips.push(idx);
        }

        trips_by_block
            .entry(block_id.clone())
            .or_default()
            .push(idx);

        trips.push(Trip {
            trip_id,
            route_id,
            service_id,
            block_id,
            stop_times,
        });
    }

    for indices in trips_by_block.values_mut() {
        indices.sort_by_key(|&i| trips[i].start_time_minutes().unwrap_or(0));
    }

    Ok(GtfsData {
        stops,
        trips,
        trips_by_block,
        weekday_trips,
        weekend_trips,
    })
}

fn parse_stops(
    archive: &mut zip::ZipArchive<Cursor<&[u8]>>,
) -> Result<HashMap<String, Stop>, Box<dyn std::error::Error + Send + Sync>> {
    let mut file = archive.by_name("stops.txt")?;
    let mut content = String::new();
    file.read_to_string(&mut content)?;

    let mut stops = HashMap::new();
    let mut reader = csv::Reader::from_reader(content.as_bytes());

    for result in reader.records() {
        let record = result?;
        let id = record.get(0).unwrap_or("").to_string();
        let name = record.get(2).unwrap_or("").to_string();
        let lat: f64 = record.get(3).unwrap_or("0").parse().unwrap_or(0.0);
        let lon: f64 = record.get(4).unwrap_or("0").parse().unwrap_or(0.0);

        stops.insert(id.clone(), Stop { id, name, lat, lon });
    }

    Ok(stops)
}

fn parse_trips(
    archive: &mut zip::ZipArchive<Cursor<&[u8]>>,
) -> Result<Vec<(String, String, String, String)>, Box<dyn std::error::Error + Send + Sync>> {
    let mut file = archive.by_name("trips.txt")?;
    let mut content = String::new();
    file.read_to_string(&mut content)?;

    let mut trips = Vec::new();
    let mut reader = csv::Reader::from_reader(content.as_bytes());

    for result in reader.records() {
        let record = result?;
        let route_id = record.get(0).unwrap_or("").to_string();
        let service_id = record.get(1).unwrap_or("").to_string();
        let trip_id = record.get(2).unwrap_or("").to_string();
        let block_id = record.get(4).unwrap_or("").to_string();

        trips.push((route_id, trip_id, service_id, block_id));
    }

    Ok(trips)
}

fn parse_stop_times(
    archive: &mut zip::ZipArchive<Cursor<&[u8]>>,
) -> Result<HashMap<String, Vec<StopTime>>, Box<dyn std::error::Error + Send + Sync>> {
    let mut file = archive.by_name("stop_times.txt")?;
    let mut content = String::new();
    file.read_to_string(&mut content)?;

    let mut stop_times: HashMap<String, Vec<StopTime>> = HashMap::new();
    let mut reader = csv::Reader::from_reader(content.as_bytes());

    for result in reader.records() {
        let record = result?;
        let trip_id = record.get(0).unwrap_or("").to_string();
        let arrival_time = record.get(1).unwrap_or("").to_string();
        let stop_id = record.get(3).unwrap_or("").to_string();
        let sequence: u32 = record.get(4).unwrap_or("0").parse().unwrap_or(0);

        let arrival_time_secs = parse_time_to_secs(&arrival_time);

        stop_times.entry(trip_id).or_default().push(StopTime {
            stop_id,
            sequence,
            arrival_time,
            arrival_time_secs,
        });
    }

    for times in stop_times.values_mut() {
        times.sort_by_key(|st| st.sequence);
    }

    Ok(stop_times)
}

fn parse_time_to_secs(time_str: &str) -> Option<u32> {
    let parts: Vec<&str> = time_str.split(':').collect();
    if parts.len() >= 2 {
        let hours: u32 = parts[0].parse().ok()?;
        let mins: u32 = parts[1].parse().ok()?;
        let secs: u32 = parts.get(2).and_then(|s| s.parse().ok()).unwrap_or(0);
        Some(hours * 3600 + mins * 60 + secs)
    } else {
        None
    }
}

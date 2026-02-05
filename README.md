# Irvine Connect GTFS Realtime

This runtime inserts trip IDs into the Passio realtime feed for the Irvine Connect shuttle service.

## Problem

The Passio realtime feed provides accurate vehicle positions, latitude, and heading but lacks accurate trip assignment. This service:

1. Downloads the static GTFS schedule
2. Polls vehicle positions every second
3. Assigns trip IDs by tracking which stops each vehicle passes
4. Serves enhanced GTFS-RT via HTTP

## Algorithm

The route is shaped like a **lollipop**—buses travel down a stem, loop around, and return on the same stem. This means the same GPS coordinates appear twice per trip, so we use **stop sequence matching** rather than simple proximity:

1. **Track Position History**: Store 20 minutes of positions per vehicle
2. **Detect Stop Visits**: When within 50m of a stop, record the visit (direction-aware for overlapping segments)
3. **Score Trips**: Match visited stops against each trip's stop sequence using LCS-inspired scoring
4. **Assign Trip**: When confidence exceeds threshold (3+ stops, >50% match)
5. **Handle Transitions**: Detect terminal arrival and switch to next trip in block

## Usage

```bash
# Run the service
cargo run

# API endpoint
curl http://localhost:8080/gtfs-rt/vehicle-positions -o feed.pb
```

## API Endpoints

| Endpoint | Method | Description |
|----------|--------|-------------|
| `/gtfs-rt/vehicle-positions` | GET | GTFS-RT protobuf feed |
| `/health` | GET | Health check |

## Data Sources

- **Static GTFS**: https://github.com/catenarytransit/irvine-connect-schedule-gtfs/releases/download/latest/gtfs.zip
- **Realtime**: https://passio3.com/irvine/passioTransit/gtfs/realtime/vehiclePositions

## Project Structure

```
src/
├── main.rs           # Entry point
├── gtfs/
│   ├── loader.rs     # Downloads and parses GTFS zip
│   └── types.rs      # Stop, Trip, GtfsData
├── matcher/
│   ├── algorithm.rs  # Trip assignment logic
│   ├── history.rs    # Vehicle state tracking
│   └── proximity.rs  # Haversine distance
├── realtime/
│   └── fetcher.rs    # 1-second polling
└── api/
    └── server.rs     # HTTP server
```

## License

MIT

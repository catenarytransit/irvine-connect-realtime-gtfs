# Irvine Connect GTFS Realtime

This runtime inserts trip IDs into the Passio realtime feed for the Irvine Connect shuttle service.

## Problem

The Passio realtime feed provides accurate vehicle positions, latitude, and heading but lacks accurate trip assignment. This service:

1. Downloads the static GTFS schedule
2. Polls vehicle positions every second
3. Assigns trip IDs by tracking which stops each vehicle passes
4. Serves enhanced GTFS-RT via HTTP

## Algorithm

The route is shaped like a **lollipop**—buses travel down a stem, loop around, and return on the same stem. The same GPS coordinates appear twice per trip (outbound and inbound), so simple proximity matching fails. We use an **HMM/Viterbi map-matching** approach that resolves directional ambiguity by finding the most probable *sequence* of stops given the observed GPS trace.

### HMM Overview

Instead of asking "is the bus at this stop?", the algorithm asks "given the full GPS history, what is the most likely sequence of stops the bus has visited?"

Each GPS observation is evaluated against candidate stops using three probability components:

1. **Emission Probability** — How likely is it that this GPS point was generated if the bus is at stop *j*? Modeled as a Gaussian over distance (`σ = 25m`), replacing the old hard 80m cutoff.

2. **Transition Probability** — How likely is a transition from stop *i* to stop *j* between consecutive GPS pings? Uses an exponential penalty on the mismatch between route distance and euclidean distance (`β = 200m`). Backward transitions return a route distance of `1e6`, making them effectively impossible.

3. **Velocity Vector Consistency** — The dot product of the GPS displacement vector and the route direction vector (stop *i* → stop *j*) is computed. If the vehicle is moving *against* the stop sequence (cos ≈ -1), the transition probability is driven to near-zero via `((1+cos)/2)²`. This is the key discriminator on the lollipop stem.

4. **Temporal Penalty** — Each stop's emission is penalized when the GPS timestamp deviates from the stop's scheduled arrival time. Within 15 minutes: no penalty. Beyond that: quadratic Gaussian decay (`σ = 20min`). This prevents spatially-perfect but temporally-wrong matches from winning.

The **Viterbi algorithm** finds the path through the stop sequence that maximizes the product of these probabilities. A pruned state window (±15 stops) keeps runtime bounded for real-time use.

### Pipeline

1. **Track Position History** (`history.rs`): Store up to 1 hour / 3600 positions per vehicle.
2. **Viterbi Scoring** (`viterbi.rs`): For each candidate trip, run the HMM to produce a spatial confidence score and per-stop matched timestamps.
3. **Time Conformance** (`algorithm.rs`): Weight the Viterbi score by schedule adherence (recency-weighted delta buckets).
4. **Global Assignment** (`algorithm.rs`): Solve a Min-Cost Max-Flow problem over all vehicles × candidate trips to find the globally optimal 1:1 assignment. Includes stability bonuses, block transition detection, and spatial fallback for unassigned vehicles.
5. **Trip Updates** (`updates.rs`): Generate GTFS-RT `TripUpdate` entities using Viterbi-matched stop timestamps, with delay propagation to future stops.

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
├── main.rs              # Entry point
├── gtfs/
│   ├── loader.rs        # Downloads and parses GTFS zip
│   └── types.rs         # Stop, StopTime, Trip, GtfsData
├── matcher/
│   ├── algorithm.rs     # Global trip assignment (MCMF, terminus detection, scoring)
│   ├── viterbi.rs       # HMM/Viterbi map-matching (emission, transition, direction, time)
│   ├── proximity.rs     # Haversine, cross-track distance, emission/transition probabilities
│   ├── history.rs       # VehicleState, position history, state persistence
│   └── updates.rs       # GTFS-RT TripUpdate generation from Viterbi-matched stops
├── realtime/
│   └── fetcher.rs       # Polling loop with proxy rotation
└── api/
    └── server.rs        # HTTP server
```

### `proximity.rs`

Low-level spatial and probabilistic primitives:

- **`haversine_distance`** — Great-circle distance between two lat/lon points.
- **`cross_track_distance`** — Perpendicular distance from a point to a segment between two stops, with along-track fraction.
- **`emission_probability`** — Gaussian `exp(-d²/2σ²) / (√(2π)σ)`. Replaces hard radius cutoffs with a continuous probability.
- **`transition_probability`** — Exponential `exp(-|d_route - d_euclidean| / β) / β`. Penalizes "teleportation" where route distance and straight-line distance diverge.

### `viterbi.rs`

The HMM/Viterbi scorer, isolated for testability:

- **`viterbi_score(positions, trip, gtfs, date_str)`** — Runs the pruned Viterbi algorithm over a slice of GPS positions against a candidate trip. Returns a `ViterbiResult` containing a normalized score and `Vec<Option<u64>>` of per-stop matched timestamps.
- Precomputes cumulative route distances from the stop sequence (no `shapes.txt` dependency).
- Incorporates four signal layers: spatial emission, route-distance transition, velocity-vector direction, and temporal schedule penalty.
- Forward-only monotonic transitions enforced structurally (backward transitions get `1e6` route distance).

### `algorithm.rs`

Orchestrates the global vehicle-to-trip assignment:

- **`perform_global_assignment`** — The main entry point called each polling cycle. For each vehicle, scores all candidate trips via `viterbi_score`, then solves a Min-Cost Max-Flow problem to find the globally optimal 1:1 vehicle↔trip assignment.
- **Terminus detection** — Identifies when a vehicle visits the terminus and classifies the visit as arriving (end of trip) or departing (start of next trip) based on surrounding stop matches.
- **Block transitions** — Detects when a vehicle completes one trip and begins the next trip in the same block, applying transition bonuses.
- **Spatial fallback** — For vehicles that didn't get assigned via MCMF, attempts a proximity-based fallback using nearby stops and active trips.


## License

MIT

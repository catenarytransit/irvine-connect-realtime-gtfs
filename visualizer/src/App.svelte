<script>
  import { onMount } from 'svelte';
  import maplibregl from 'maplibre-gl';

  let map;
  let mapContainer;
  let vehicleStates = [];
  let selectedVehicle = null;
  let error = null;

  const TERMINUS_LON = -117.732860708;
  const TERMINUS_LAT = 33.656831556;
  const TERMINUS_RADIUS_METERS = 80.0;

  // Simple approximation: 1 degree latitude is ~111,139 meters. 
  // At latitude 33.65, 1 degree longitude is ~111,139 * cos(33.65) ~= 92,500 meters.
  function createGeoJSONCircle(center, radiusInMeters, points = 64) {
      const coords = {
          latitude: center[1],
          longitude: center[0]
      };

      const km = radiusInMeters / 1000;
      const ret = [];
      const distanceX = km / (111.320 * Math.cos(coords.latitude * Math.PI / 180));
      const distanceY = km / 110.574;

      let theta, x, y;
      for (let i = 0; i < points; i++) {
          theta = (i / points) * (2 * Math.PI);
          x = distanceX * Math.cos(theta);
          y = distanceY * Math.sin(theta);

          ret.push([coords.longitude + x, coords.latitude + y]);
      }
      ret.push(ret[0]);

      return {
          type: "Feature",
          geometry: {
              type: "Polygon",
              coordinates: [ret]
          }
      };
  }

  async function fetchDebugState() {
      try {
          const res = await fetch('http://localhost:8080/debug/state');
          if (!res.ok) throw new Error('API error: ' + res.status);
          const data = await res.json();
          vehicleStates = data;
          error = null;
          updateMap();
      } catch (err) {
          error = err.message;
      }
  }

  function updateMap() {
      if (!map || !map.isStyleLoaded()) return;

      const vehicleGeojson = {
          type: "FeatureCollection",
          features: vehicleStates.map(v => {
              const lastPos = v.position_history[v.position_history.length - 1];
              if (!lastPos) return null;
              return {
                  type: "Feature",
                  geometry: { type: "Point", coordinates: [lastPos.lon, lastPos.lat] },
                  properties: { ...v }
              };
          }).filter(Boolean)
      };

      map.getSource('vehicles').setData(vehicleGeojson);

      if (selectedVehicle) {
          const v = vehicleStates.find(x => x.vehicle_id === selectedVehicle.vehicle_id);
          if (v) {
              const breadcrumbsGeojson = {
                  type: "FeatureCollection",
                  features: v.position_history.map(p => ({
                      type: "Feature",
                      geometry: { type: "Point", coordinates: [p.lon, p.lat] },
                      properties: p
                  }))
              };
              
              const now = Date.now() / 1000;
              const maxAge = 600; // 10 minutes

              const breadcrumbsLineGeojson = {
                  type: "FeatureCollection",
                  features: []
              };

              for (let i = 0; i < v.position_history.length - 1; i++) {
                  const p1 = v.position_history[i];
                  const p2 = v.position_history[i+1];
                  const age = now - p2.timestamp;
                  
                  breadcrumbsLineGeojson.features.push({
                      type: "Feature",
                      geometry: { type: "LineString", coordinates: [[p1.lon, p1.lat], [p2.lon, p2.lat]] },
                      properties: { age }
                  });
              }

              if (map.getSource('breadcrumbs')) map.getSource('breadcrumbs').setData(breadcrumbsGeojson);
              if (map.getSource('breadcrumbs-line')) map.getSource('breadcrumbs-line').setData(breadcrumbsLineGeojson);
              selectedVehicle = v;
          }
      }
  }

  onMount(() => {
      map = new maplibregl.Map({
          container: mapContainer,
          style: 'https://maps.catenarymaps.org/dark-style.json',
          center: [TERMINUS_LON, TERMINUS_LAT],
          zoom: 14
      });

      map.on('load', () => {
          // Terminus circle
          map.addSource('terminus', {
              type: 'geojson',
              data: createGeoJSONCircle([TERMINUS_LON, TERMINUS_LAT], TERMINUS_RADIUS_METERS)
          });
          map.addLayer({
              id: 'terminus-fill',
              type: 'fill',
              source: 'terminus',
              paint: {
                  'fill-color': 'rgba(255, 0, 0, 0.2)',
                  'fill-outline-color': 'red'
              }
          });
          
          map.addSource('vehicles', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
          map.addLayer({
              id: 'vehicles-layer',
              type: 'circle',
              source: 'vehicles',
              paint: {
                  'circle-radius': 8,
                  'circle-color': '#007cbf',
                  'circle-stroke-width': 2,
                  'circle-stroke-color': '#fff'
              }
          });

          map.addSource('breadcrumbs-line', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
          map.addLayer({
              id: 'breadcrumbs-line-layer',
              type: 'line',
              source: 'breadcrumbs-line',
              paint: {
                  'line-width': 2,
                  'line-color': [
                      'interpolate',
                      ['linear'],
                      ['get', 'age'],
                      0, '#ff0000',     // 0 seconds ago -> red
                      300, '#ffff00',   // 5 minutes ago -> yellow
                      600, '#00ff00',   // 10 minutes ago -> green
                      1800, '#0000ff'   // 30+ minutes ago -> blue
                  ]
              }
          });

          map.addSource('breadcrumbs', { type: 'geojson', data: { type: 'FeatureCollection', features: [] } });
          map.addLayer({
              id: 'breadcrumbs-layer',
              type: 'circle',
              source: 'breadcrumbs',
              paint: {
                  'circle-radius': 4,
                  'circle-color': '#fc4c02'
              }
          });

          map.on('click', 'vehicles-layer', (e) => {
              const props = e.features[0].properties;
              selectedVehicle = vehicleStates.find(v => v.vehicle_id === props.vehicle_id);
              updateMap();
          });
      });

      fetchDebugState();
      const interval = setInterval(fetchDebugState, 2000);

      return () => clearInterval(interval);
  });
</script>

<style>
  :global(body) { margin: 0; padding: 0; font-family: sans-serif; width: 100vw; height: 100vh; overflow: hidden; }
  .map-container { position: absolute; top: 0; bottom: 0; left: 0; right: 0; }
  .sidebar { position: absolute; top: 10px; left: 10px; width: 400px; max-height: calc(100vh - 50px); padding: 1rem; overflow-y: auto; background: rgba(255, 255, 255, 0.95); border-radius: 8px; box-shadow: 0 2px 10px rgba(0,0,0,0.2); z-index: 1; }
  h2 { margin-top: 0; }
  .box { background: white; padding: 10px; border-radius: 8px; margin-bottom: 10px; box-shadow: 0 2px 4px rgba(0,0,0,0.1); }
  p { font-size: 14px; color: #444; }
  .candidate { padding: 4px 8px; border-bottom: 1px solid #eee; font-family: monospace; font-size: 13px; }
  .candidate:last-child { border-bottom: none; }
  .badge { display: inline-block; padding: 2px 6px; border-radius: 4px; background: #eee; font-size: 12px; margin-right: 4px;}
</style>

<div bind:this={mapContainer} class="map-container"></div>
<div class="sidebar">
  <h2>Debug Internal State</h2>
  {#if error}
      <p style="color:red">Error: {error}</p>
  {/if}

  {#if selectedVehicle}
      <div class="box">
          <h3>Vehicle: {selectedVehicle.vehicle_id}</h3>
          <p>Assigned Trip: {selectedVehicle.assigned_trip_id || 'None'}</p>
          <p>Confidence: {(selectedVehicle.trip_confidence * 100).toFixed(1)}%</p>
          <p>Previous Trip: {selectedVehicle.previous_trip_id || 'None'}</p>

          <h4>Last Terminus Departure</h4>
          <p>
              {selectedVehicle.last_terminus_departure 
                  ? new Date(selectedVehicle.last_terminus_departure * 1000).toLocaleTimeString() 
                  : 'N/A'}
          </p>

          <h4>Matched Stops</h4>
          <div style="font-family: monospace; font-size: 12px; max-height: 100px; overflow-y: auto;">
              {#each selectedVehicle.visited_stops as stop, i}
                  <div>
                      - {stop} at {selectedVehicle.stop_visit_timestamps[i] ? new Date(selectedVehicle.stop_visit_timestamps[i][1] * 1000).toLocaleTimeString() : '?'}
                  </div>
              {/each}
          </div>

          <h4>Trip Candidates</h4>
          <div style="background: #f0f0f0; padding: 10px; border-radius: 4px;">
              {#each selectedVehicle.debug_trip_candidates || [] as [trip_id, score]}
                  <div class="candidate">
                      {trip_id} <span style="float:right">score: {score.toFixed(3)}</span>
                  </div>
              {:else}
                  <div>No candidates this cycle</div>
              {/each}
          </div>
      </div>
  {:else}
      <p>Select a vehicle on the map to see internal algorithm state.</p>
  {/if}
  
  <h3>All Vehicles ({vehicleStates.length})</h3>
  <div class="box">
      {#each vehicleStates as v}
          <div style="margin-bottom: 8px; cursor: pointer; color: blue;" on:click={() => { selectedVehicle = v; updateMap(); map.flyTo({center: [v.position_history[v.position_history.length-1].lon, v.position_history[v.position_history.length-1].lat], zoom: 15});}}>
              #{v.vehicle_id} - Trip: {v.assigned_trip_id || '?'}
          </div>
      {/each}
  </div>
</div>

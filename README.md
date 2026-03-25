# Route-Aware OSM Analysis for Autonomous Driving Applications

A Python workflow that builds drivable routes from OpenStreetMap data, extracts detailed road attributes, computes geometric and topological metrics, and transforms the results into structured outputs suitable for AV/CAV research and simulation.

---

## Overview

The script `osm_route_analysis.py` performs end-to-end route analysis:

- Builds a drivable route through a start point, optional intermediate waypoints, and an end point using OSM road network data.
- Extracts per-segment road attributes: lane count, speed limit, road type, and direction.
- Computes geometric properties for each segment: bearing, curvature, and sinuosity.
- Detects and characterizes every intersection along the route, including connected roads, traffic controls, and position.
- Measures waypoint density as a proxy for road geometry complexity.
- Transforms all raw data into a structured AV/CAV decision layer with driving states, advisories, and upcoming events.

---

## Input Definition

Routes are defined as a **start point**, up to three **waypoints**, and an **end point**, all expressed as `(latitude, longitude)` pairs.

Three routes are predefined in the `ROUTES` dictionary at the top of the script:

| ID | Name |
|----|------|
| 1  | FM 1362 to FM 2000 |
| 2  | FM 908 to CR 316 |
| 3  | US 190 to FM 2038 to FM 244 |

Each input point is snapped to the nearest OSM node in the downloaded road network before routing begins. Snap distances are recorded in `snap_summary`.

---

## Core Features

### Route Generation

- Downloads the OSM road network for a bounding polygon around all input points using OSMnx.
- Computes the shortest path between consecutive snapped nodes (minimizing travel time, with length as fallback).
- Concatenates individual leg paths into a single ordered list of route nodes.
- Automatically expands the search polygon and retries if routing fails.

### Segment-Level Analysis

Each consecutive pair of route nodes forms one segment. For every segment the script extracts or infers:

- `primary_highway`: road classification (motorway, primary, secondary, residential, etc.)
- `lanes_count`: number of lanes
- `maxspeed_kph`: speed limit in km/h
- `oneway_bool` / `direction`: whether the road is one-way or two-way
- `travel_time_s`: estimated travel time for the segment
- `bearing_deg`: compass bearing of the segment

**Inference fallbacks** — when OSM tags are absent, values are estimated and flagged:

| Attribute | Inference basis | Confidence |
|-----------|----------------|------------|
| Lane count | Road type or speed limit | 0.5 – 0.8 |
| Speed limit | Road classification | 0.6 |

Typical inferred speed limits by road class:

| Road type | Approximate speed |
|-----------|------------------|
| motorway | 100 – 120 km/h |
| trunk | 100 km/h |
| primary | 80 – 90 km/h |
| secondary | 60 – 80 km/h |
| tertiary | 60 km/h |
| residential | 30 – 50 km/h |

The fields `lane_inference_source`, `lane_confidence`, `speed_inference_source`, and `speed_confidence` record how each value was obtained. Confidence values are bounded to `[0.0, 1.0]`.

### Curvature Analysis

Curvature is the absolute bearing change between consecutive segments:

| Curvature | Classification |
|-----------|---------------|
| < 10 deg  | straight |
| 10 – 45 deg | moderate |
| > 45 deg  | sharp |

Each segment records `bearing_deg`, `curvature_deg`, and `curvature_class`. The first segment always has `curvature_deg = 0`.

### Waypoint Density

Waypoint density measures how many coordinate vertices a segment's polyline contains relative to its length. A dense polyline indicates a geometrically complex road; a sparse one indicates a straight or simplified road.

| Field | Description |
|-------|-------------|
| `geometry_point_count` | Number of coordinate vertices in the segment polyline |
| `waypoint_density_per_100m` | Vertex count per 100 m of road length |
| `waypoint_density_per_km` | Vertex count per km of road length |

Density is classified as:

| Classification | Threshold |
|---------------|-----------|
| low | < 5 per km |
| medium | 5 – 15 per km |
| high | > 15 per km |

Segments shorter than 1 m return `null` density values to avoid numerical noise.

A summary across all segments is stored in `waypoint_density_summary`:

```json
"waypoint_density_summary": {
  "avg_density_per_km": ...,
  "max_density_per_km": ...,
  "min_density_per_km": ...
}
```

### Intersection Analysis

An intersection is any route node whose undirected degree is 3 or greater. For each intersection the script records:

| Field | Description |
|-------|-------------|
| `node_id` | OSM node identifier |
| `node_degree` | Number of edges connected to the node |
| `connected_edge_count` | Unique edges incident to the node |
| `connected_highway_types` | Sorted list of road types on connected edges |
| `connected_road_names` | Sorted list of road names on connected edges |
| `connected_oneway_count` | Number of connected one-way edges |
| `connected_twoway_count` | Number of connected two-way edges |
| `is_signalized` | Traffic signal within 30 m |
| `is_stop_controlled` | Stop sign within 30 m |
| `has_crossing_nearby` | Pedestrian crossing within 30 m |
| `route_position_index` | Index of this node in the ordered route node list |
| `distance_from_start_m` | Cumulative route distance from the start |
| `is_input_point` | Whether this node is a snapped input point |
| `input_point_type` | `start`, `waypoint_N`, `end`, or `none` |

### Driving State (per segment)

Each segment carries a `driving_state` object:

```json
"driving_state": {
  "geometry": "straight | left_turn | right_turn | turn",
  "topology": "intersection | road_segment",
  "control": "traffic_light | stop_sign | pedestrian_crossing | none"
}
```

- `geometry` is derived from `curvature_deg` and the signed bearing change versus the previous segment.
- `topology` is set to `intersection` when the segment midpoint is within 40 m of a known intersection.
- `control` is assigned by proximity (within 30 m) to traffic signals, stop signs, or pedestrian crossings, checked in that priority order.

### Corridor Context

A buffer of configurable width (default 30 m) is drawn around the route, and OSM features within that corridor are queried and classified:

- Traffic controls: signals, stop signs, give-way signs, crossings, traffic calming, roundabouts, barriers.
- Context: bus stops, bike lanes, schools, hospitals, parking, transit stations.
- Land use and buildings.

Summary counts and land-use area fractions are stored in `corridor_summary`.

---

## Output Files

For each run three files are written to `output/`:

| File | Description |
|------|-------------|
| `<route_name>.json` | Full raw OSM analysis output |
| `<route_name>_cav.json` | Structured AV/CAV decision layer |
| `<route_name>_map.html` | Interactive Folium map (requires `folium`) |

Route names are sanitized to underscores (e.g., `FM_908_to_CR_316`).

### Raw OSM Output (`<route_name>.json`)

Top-level keys:

| Key | Contents |
|-----|----------|
| `route_name` | Name of the selected route |
| `inputs` | Start, waypoints, end coordinates and corridor width |
| `route_metrics` | Length (km, miles) and estimated travel time |
| `points_of_interest` | Intersections, signals, stops, crossings, bike lanes |
| `intersection_details` | Per-intersection records (see above) |
| `edge_attributes` | Per-segment records including driving state and density |
| `waypoint_density_summary` | Aggregate density statistics |
| `curvature_analysis` | Per-segment bearing and curvature |
| `corridor_summary` | Feature counts and land-use breakdown in the corridor |
| `snap_summary` | Input-to-OSM node snapping results |
| `route_geometry_topology` | Route length, segment count, intersection density, leg summaries |
| `edge_attribute_summary` | Aggregate highway, speed, lane, direction, and bike-infra statistics |
| `turn_restrictions` | Turn-restriction relations from Overpass API |

### CAV Output (`<route_name>_cav.json`)

Transforms raw OSM data into structured driving intelligence. Top-level keys:

```json
{
  "route_name": "...",
  "route_summary": { ... },
  "segments": [ ... ],
  "intersections": [ ... ],
  "upcoming_events": [ ... ],
  "density_analysis": { ... }
}
```

**`route_summary`**

```json
{
  "total_length_km": ...,
  "estimated_time_min": ...,
  "segment_count": ...,
  "intersection_count": ...,
  "difficulty_level": "low | medium | high"
}
```

Difficulty is computed from the number of sharp curves and the intersection density per km.

**`segments`** — one record per segment:

```json
{
  "segment_id": ...,
  "length_m": ...,
  "road_info":  { "type": ..., "lanes": ..., "speed_limit_kph": ... },
  "geometry":   { "type": "straight | moderate | sharp", "curvature_deg": ... },
  "density":    { "waypoint_density_per_km": ..., "classification": "low | medium | high" },
  "state":      { "turn_type": "straight | left | right", "road_type": "...", "recommended_action": "..." },
  "advisory":   { "speed_advisory_kph": ..., "reason": "..." }
}
```

Advisory speeds: sharp turn → 50 % of speed limit, moderate curve → 70 %, straight → unchanged.

**`intersections`** — one record per intersection:

```json
{
  "node_id": ...,
  "distance_from_start_m": ...,
  "type": "3-way | 4-way",
  "connected_roads": [ ... ],
  "control": "signal | stop | none",
  "state":    { "event": "approaching_intersection", "priority": "low | medium | high" },
  "advisory": { "action": "stop | yield | proceed", "reason": "..." }
}
```

**`upcoming_events`** — combined list of turn and intersection events sorted by `distance_ahead_m`:

```json
{
  "distance_ahead_m": ...,
  "type": "turn | intersection",
  "severity": "low | moderate | high",
  "recommended_speed_kph": ...,   // turn events only
  "action": "stop | yield | proceed"  // intersection events only
}
```

**`density_analysis`**:

```json
{
  "avg_density_per_km": ...,
  "max_density_per_km": ...,
  "min_density_per_km": ...,
  "high_density_segment_count": ...
}
```

---

## Map Visualization

The HTML map (`<route_name>_map.html`) requires `folium` and includes:

- Route polyline in blue.
- Start (green), waypoint (blue), and end (red) markers.
- Toggle layers for traffic signals, stop signs, crossings, and bike lanes.

---

## Technologies

| Library | Purpose |
|---------|---------|
| Python 3.10+ | Core language |
| OSMnx | OSM graph download, nearest-node snapping |
| NetworkX | Shortest-path routing |
| GeoPandas | Spatial operations, CRS projection, corridor buffering |
| Shapely | Geometry construction and manipulation |
| Requests | Overpass API queries for turn restrictions |
| Folium (optional) | Interactive HTML map export |

---

## Installation

```bash
python -m venv .venv
```

Activate the environment:

- macOS / Linux: `source .venv/bin/activate`
- Windows PowerShell: `.venv\Scripts\Activate.ps1`

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## How to Run

```bash
python osm_route_analysis.py
```

Select a route when prompted:

```text
Available routes:
1: FM 1362 to FM 2000
2: FM 908 to CR 316
3: US 190 to FM 2038 to FM 244
Select route (1, 2, or 3):
```

Output files are written to `output/`:

```text
output/
  <route_name>.json
  <route_name>_cav.json
  <route_name>_map.html
```

---

## Configurable Parameters

At the top of `osm_route_analysis.py`:

| Parameter | Default | Description |
|-----------|---------|-------------|
| `ROUTES` | — | Predefined route definitions |
| `CORRIDOR_WIDTH_M` | `30.0` | Corridor buffer width in metres |
| `OUTPUT_DIR` | `output` | Output folder |

---

## Notes

- All JSON exports sanitize `NaN` and `inf` values to `null` for strict JSON compliance.
- Missing OSM tags (lanes, speed) are inferred where possible; source and confidence are always recorded alongside the inferred value.
- Turn-restriction queries use the Overpass API; failures are captured in the output instead of aborting the run.
- The project is designed for AV/CAV research and is structured to be extended with additional analysis layers.

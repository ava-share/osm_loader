## Route-Aware OpenStreetMap Analysis

This repository contains a single workflow script, `osm_route_analysis.py`, for predefined multi-waypoint route analysis using OpenStreetMap (OSM).

Current behavior:

- Uses predefined routes (`start + 3 waypoints + end`) from `ROUTES`.
- Prompts the user to select route `1`, `2`, or `3`.
- Builds a drivable route via waypoints.
- Extracts edge-level OSM attributes and derived metrics.
- Queries corridor features and builds explicit points of interest.
- Exports one JSON file per selected route.
- Optionally exports one interactive Folium HTML map.

---

## Installation

Python 3.10+ is recommended.

```bash
python -m venv .venv
```

Activate environment:

- Windows PowerShell:

```powershell
.venv\Scripts\Activate.ps1
```

- macOS / Linux:

```bash
source .venv/bin/activate
```

Install dependencies:

```bash
pip install -r requirements.txt
```

---

## Usage

Run:

```bash
python osm_route_analysis.py
```

You will be prompted:

```text
Available routes:
1: FM 1362 to FM 2000
2: FM 908 to CR 316
3: US 190 to FM 2038 to FM 244
Select route (1, 2, or 3):
```

The script then runs end-to-end and writes output files in `output/`.

---

## Configurable Parameters

At the top of `osm_route_analysis.py`:

- `ROUTES`: predefined route definitions
- `CORRIDOR_WIDTH_M`: corridor width in meters (default `30.0`)
- `OUTPUT_DIR`: output folder (default `output`)

---

## Output Files

For each selected route:

- `output/<route_name>.json`
- `output/<route_name>_map.html` (if Folium is installed)

Route names are sanitized with underscores (for example: `FM_1362_to_FM_2000.json`).

---

## JSON Structure

Each route JSON includes:

- `route_name`
- `inputs`
- `route_metrics`
  - `length_km`
  - `length_miles`
  - `travel_time_min`
- `points_of_interest`
  - `intersections`
  - `traffic_signals`
  - `stop_signs`
  - `crossings`
  - `bike_lanes`
- `edge_attributes`
- `curvature_analysis`
- `corridor_summary`
- `snap_summary`
- `route_geometry_topology`
- `edge_attribute_summary`
- `turn_restrictions`

### Edge Attributes (per segment)

Each entry in `edge_attributes` includes fields such as:

- `segment_id`, `from_node`, `to_node`, `segment_length_m`
- `bearing_deg`, `curvature_deg`, `curvature_class`
- `primary_highway`, `oneway_bool`, `direction`
- `lanes_count`, `maxspeed_kph`, `travel_time_s`
- `lane_inference_source`, `lane_confidence`
- `speed_inference_source`, `speed_confidence`
- `major_interpreted_notes`

### Inference and Confidence

If direct OSM tags are missing:

- `lanes_count` may be inferred from speed/highway type.
- `maxspeed_kph` may be inferred from highway type.
- Source and confidence are recorded in:
  - `lane_inference_source`, `lane_confidence`
  - `speed_inference_source`, `speed_confidence`

Confidence values are bounded to `0.0` - `1.0`.

---

## Map Visualization

The HTML map includes:

- Route polyline.
- Start (green), waypoints (blue), end (red) markers.
- POI layers:
  - traffic signals (red)
  - stop signs (orange)
  - crossings (purple)
  - bike lanes (green)

---

## Reliability Notes

- JSON export sanitizes `NaN` values to `null`.
- Missing geometry/OSM fields are handled with safe fallbacks where implemented.
- Turn restrictions are queried from Overpass when enabled; failures are captured in output instead of crashing the full run.
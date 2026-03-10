## Waypoints-based OpenStreetMap Route Analysis

This repository provides a **single, self-contained Python workflow** for doing route-aware analysis on OpenStreetMap (OSM) data.

Given a start point, optional intermediate waypoints, and an end point, the script will:

- **Build a drivable route** through the given points using OSM street networks.
- **Extract traversed edge attributes** (e.g., highway type, lanes, maxspeed, sidewalks, cycleways).
- **Construct a metric corridor** around the route and **query nearby context / safety features** (signals, crossings, schools, hospitals, parking, etc.).
- **Compute summary statistics** about geometry, speed limits, intersections, sidewalks, and corridor context.
- **Export ready-to-use outputs** as CSV/JSON/GeoJSON and an optional interactive HTML map.

All core logic lives in `osm_route_analysis.py`.

---

## Installation

**Python version**: Recommended Python 3.10+ with a fresh virtual environment.

1. **Create and activate a virtual environment** (example with `python` pointing to your desired version):

```bash
python -m venv .venv
source .venv/bin/activate  # macOS / Linux
# .venv\Scripts\activate   # Windows PowerShell
```

2. **Install the required dependencies** (binary wheels are important for geopandas / shapely / pyproj):

```bash
pip install -r requirements.txt
```

If imports fail at runtime, the script will raise a clear error pointing back to `requirements.txt`.

---

## Usage

The main entry point is the `main` function in `osm_route_analysis.py`, which is also invoked when the script is run directly.

### 1. Configure the route

At the top of `osm_route_analysis.py` you can edit:

- `START`: `(lat, lon)` tuple for the origin.
- `MIDDLE`: list of `(lat, lon)` waypoints (can be an empty list).
- `END`: `(lat, lon)` tuple for the destination.
- `CORRIDOR_WIDTH_M`: corridor half-width in meters around the route used for querying nearby features.
- `OUTPUT_DIR`: directory where outputs will be written (default `output/`).

These are simple Python variables, so you can either:

- Edit them directly in the file and run the script as a one-off analysis, or
- Import `main` from another Python module and call it with your own parameters.

### 2. Run from the command line

After activating your virtual environment and configuring the inputs:

```bash
python osm_route_analysis.py
```

By default this will:

- Download a drivable OSM network for the search area.
- Snap your input points to the nearest network nodes.
- Build a route (using `travel_time`, falling back to `length` if needed).
- Query corridor features around the route.
- Summarize route geometry, attributes, and corridor context.
- Export files to `OUTPUT_DIR`.

You can also call the workflow programmatically, e.g.:

```python
from osm_route_analysis import main

result = main(
    start=(lat1, lon1),
    middle=[(lat_mid, lon_mid)],
    end=(lat2, lon2),
    corridor_width_m=30.0,
    output_dir="output",
    query_turn_restrictions=True,
    export_map=True,
)
```

The returned `result` dictionary includes the route summary, GeoDataFrames, export paths, and a human-readable text report.

---

## Outputs

By default, the workflow writes to `OUTPUT_DIR` (default `output/`) the following:

- **`route_segments.csv`**: ordered route segments with geometry (WKT) and a rich set of OSM attributes plus derived metrics (bearing, length, parsed maxspeed, lanes, etc.).
- **`route_summary.json`**: structured JSON with:
  - Input points and corridor configuration.
  - Snap information and search area details.
  - Route geometry/topology metrics (length, intersections, sinuosity, leg summaries).
  - Edge attribute summaries (speed limits, lane distributions, sidewalks, oneway proportions).
  - Corridor feature summaries (traffic controls, land use, nearby schools/hospitals, etc.).
  - Optional turn-restriction statistics from Overpass.
- **`route_features.geojson`**: combined GeoJSON layer including:
  - Route edges, the route centerline, and the corridor polygon.
  - All queried corridor features, with a standardized `feature_type` and `feature_group`.
- **`route_map.html`** (optional): interactive Folium web map with:
  - Route polyline.
  - Corridor features symbolized by type (signals, crossings, transit stops, schools, hospitals, parking, etc.).
  - Start / end / waypoint markers and a layer control.

These outputs are designed to be easily consumed by GIS tools (e.g., QGIS), notebooks (via GeoPandas), or downstream analysis scripts.

---

## Turn Restrictions (Optional)

If `query_turn_restrictions=True` in `main`, the script will query the Overpass API for turn-restriction relations in the corridor bounding box and add a small summary of restriction counts into the JSON summary.

If the Overpass request fails or times out, the error is recorded in the summary instead of failing the entire workflow.

---

## Notes and Limitations

- OSM data coverage and tagging quality can vary by region; missing tags (e.g., `maxspeed`, `sidewalk`) are explicitly tracked in derived notes and summaries.
- For very long routes or large corridors, network download and corridor queries can be slow and memory-intensive.
- The script is intentionally written as a **single-file workflow** to make it easy to reuse in notebooks or other projects—feel free to adapt / modularize for your own use cases.


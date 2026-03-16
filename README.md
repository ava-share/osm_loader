## OpenStreetMap (OSM) Route Analysis + Route Planning

This repository contains a small set of Python tools for building and analyzing routes on OpenStreetMap:

- **`osm_route_analysis.py`**: end-to-end **route + corridor analysis** (exports CSV/JSON/GeoJSON + optional maps)
- **`dc_point_picker.py`**: a local web map to **pick START/END points in Washington, DC** and save them to JSON
- **`route_planner.py`**: downloads a network, generates **K candidate routes**, scores them by **route complexity**, and exports a comparison map + JSON
- **`route_complexity.py`**: computes a single **route complexity report** from `osm_route_analysis.py` outputs

---

## Installation

- **Python**: 3.10+ recommended

Create and activate a virtual environment, then install dependencies:

```bash
python -m venv .venv
source .venv/bin/activate  # macOS / Linux
# .venv\Scripts\activate   # Windows PowerShell
pip install -r requirements.txt
```

Notes:

- **`folium` is optional**. If it’s not installed, the scripts will still run but some HTML map outputs will be skipped.

---

## Quick start (DC workflow)

### 1. Pick START/END points (DC)

This opens a local web map and saves your selection to `output/selected_points.json`.

```bash
python dc_point_picker.py
```

Controls (in the opened map):

- **S**: set **START** at the mouse cursor
- **E**: set **END** at the mouse cursor
- **Enter** (or on-screen ENTER button): save

### 2. Compare K candidate routes and pick the easiest

This loads `output/selected_points.json`, generates candidate routes, scores each route, writes JSON summaries, and creates `output/route_planning_map.html`.

```bash
python route_planner.py --k 3 --output-dir output
```

Outputs from this step (in `output/`):

- **`route_planning_map.html`**: candidate routes + highlighted “best” (lowest score)
- **`routes_comparison.json`**: minimal comparison + selected best route id
- **`routes_complexity.json`**: detailed per-route complexity metrics

### 3. (Optional) Run full corridor analysis on a specific route

For full exports (segments CSV, corridor features, GeoJSON layers, etc.), run the core analysis programmatically:

```python
from osm_route_analysis import main

result = main(
    start=(lat1, lon1),
    middle=[],
    end=(lat2, lon2),
    corridor_width_m=30.0,
    output_dir="output",
    query_turn_restrictions=True,
    export_map=True,
)
```

---

## Core analysis (`osm_route_analysis.py`)

### What it does

Given **start**, optional **waypoints**, and **end**, the workflow will:

- **Build a drivable route** using OSMnx + NetworkX shortest paths (prefers `travel_time`, falls back to `length`).
- **Extract per-edge attributes** (e.g., `highway`, `lanes`, `maxspeed`, `sidewalk`, `cycleway`) plus derived metrics.
- **Build a corridor buffer** around the route and query nearby context/safety features (signals, crossings, schools, hospitals, parking, etc.).
- **Export** CSV/JSON/GeoJSON, and (if `folium` is installed) HTML maps.

### Outputs (written to `output_dir`, default `output/`)

- **`route_segments.csv`**: ordered route segments (geometry as WKT) with OSM attributes + derived fields (bearing, parsed speed, oneway, etc.).
- **`corridor_features.csv`**: corridor features as a flat table (includes `feature_type`, `feature_group`, and representative `latitude`/`longitude`).
- **`route_summary.json`**: structured JSON summary with:
  - Inputs + snapping info
  - Route geometry/topology metrics (length, intersections, sinuosity, leg summaries)
  - Attribute summaries (speed limits, lanes, sidewalks, oneway proportions, bike infra)
  - Corridor feature summaries (traffic controls, landuse coverage, nearby schools/hospitals, etc.)
  - Optional Overpass turn-restriction counts (or a recorded failure message)
- **`route_features.geojson`**: combined layer containing route edges, route line, corridor polygon, and corridor features.
- **`route_map.html`** (optional): interactive map (route + corridor features).
- **`route_map_enhanced.html`** (optional): enhanced map using exported files (segment difficulty coloring + a floating complexity panel when available).

---

## Route complexity (`route_complexity.py`)

This script computes a single `route_complexity.json` from `osm_route_analysis.py` outputs.

Default usage (expects outputs in `results/`, but will fall back to `output/` if `results/` doesn’t exist):

```bash
python route_complexity.py --results-dir output
```

It uses these components:

- intersection density (per km)
- sinuosity proxy
- traffic signal density (per km)
- crossing density (per km)

---

## Notes and limitations

- **OSM tagging varies by region**; missing tags (e.g., `maxspeed`, `sidewalk`) are expected and tracked in summaries.
- **Large routes/corridors can be slow** (network download + corridor queries).
- Overpass turn-restriction queries can fail/time out; failures are recorded in the JSON instead of aborting the run.


#!/usr/bin/env python3
"""
Route-aware OpenStreetMap analysis workflow.

This script:
1. Builds a drivable route through start -> optional middle waypoints -> end.
2. Extracts traversed edge attributes from OSM.
3. Builds a metric corridor around the route and queries nearby OSM context/safety features.
4. Produces route-level summaries and segment-level outputs.
5. Exports CSV/JSON/GeoJSON and optionally an interactive HTML map.
"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Iterable, List, Optional, Sequence, Tuple

try:
    import geopandas as gpd
    import networkx as nx
    import osmnx as ox
    import pandas as pd
    import requests
except Exception as exc:
    raise RuntimeError(
        "Failed to import required geospatial dependencies. "
        "Create a virtual environment and install /Users/ylin/Research/osm/requirements.txt "
        "to ensure compatible binary wheels."
    ) from exc
from shapely.geometry import LineString, Point, Polygon


# ---------------------------------------------------------------------------
# Input parameters (predefined routes)
# ---------------------------------------------------------------------------
ROUTES: Dict[str, Dict[str, Any]] = {
    "1": {
        "name": "FM 1362 to FM 2000",
        "start": (30.5913233, -96.6204616),
        "waypoints": [
            (30.70138, -96.64029),
            (30.69026, -96.66475),
            (30.6208, -96.6866),
        ],
        "end": (30.553535, -96.677703),
    },
    "2": {
        "name": "FM 908 to CR 316",
        "start": (30.555226, -96.746941),
        "waypoints": [
            (30.554, -96.79427),
            (30.5268, -96.8477),
            (30.51229, -96.79985),
        ],
        "end": (30.4958, -96.74689),
    },
    "3": {
        "name": "US 190 to FM 2038 to FM 244",
        "start": (30.771487, -96.076427),
        "waypoints": [
            (30.7203, -96.0902),
            (30.7185, -96.1589),
            (30.7815, -96.2649),
        ],
        "end": (30.696641, -96.35882),
    },
}


# Corridor width in meters for nearby feature extraction
CORRIDOR_WIDTH_M: float = 30.0

# Output directory
OUTPUT_DIR: str = "output"


ROUTE_EDGE_COLUMNS = [
    "name",
    "highway",
    "length",
    "oneway",
    "lanes",
    "lanes:forward",
    "lanes:backward",
    "turn:lanes",
    "maxspeed",
    "surface",
    "width",
    "bridge",
    "tunnel",
    "access",
    "service",
    "junction",
    "sidewalk",
    "cycleway",
    "cycleway:left",
    "cycleway:right",
    "shoulder",
]

DEFAULT_CORRIDOR_TAGS: Dict[str, Any] = {
    "highway": ["traffic_signals", "stop", "give_way", "crossing", "bus_stop", "mini_roundabout", "cycleway"],
    "crossing": True,
    "traffic_calming": True,
    "railway": ["level_crossing", "crossing"],
    "junction": ["roundabout"],
    "cycleway": True,
    "cycleway:left": True,
    "cycleway:right": True,
    "barrier": True,
    "amenity": ["school", "hospital", "parking", "bus_station"],
    "public_transport": ["platform", "stop_position", "station"],
    "landuse": True,
    "building": True,
    "healthcare": ["hospital"],
    "parking": True,
}


@dataclass
class BuildRouteResult:
    graph: nx.MultiDiGraph
    input_points: List[Tuple[float, float]]
    snapped_nodes: List[int]
    snapped_info: pd.DataFrame
    route_nodes: List[int]
    leg_paths: List[List[int]]
    search_polygon_wgs84: Polygon
    search_buffer_m: float


def _is_null(value: Any) -> bool:
    if value is None:
        return True
    try:
        return bool(pd.isna(value))
    except (TypeError, ValueError):
        return False


def _to_list(value: Any) -> List[Any]:
    if _is_null(value):
        return []
    if isinstance(value, list):
        return value
    if isinstance(value, tuple):
        return list(value)
    if isinstance(value, set):
        return sorted(list(value))
    return [value]


def _normalize_value(value: Any) -> Optional[str]:
    if _is_null(value):
        return None
    if isinstance(value, (list, tuple, set)):
        flat = [str(v) for v in value if not _is_null(v)]
        return "|".join(flat) if flat else None
    if isinstance(value, dict):
        return json.dumps(value, ensure_ascii=True, sort_keys=True)
    return str(value)


def _first_text(value: Any) -> Optional[str]:
    values = _to_list(value)
    for v in values:
        if not _is_null(v):
            return str(v)
    return None


def _ensure_projected_crs(geodata: gpd.GeoSeries) -> str:
    estimated = geodata.estimate_utm_crs()
    if estimated is None:
        return "EPSG:3857"
    return str(estimated)


def _build_search_polygon(points: Sequence[Tuple[float, float]], buffer_m: float) -> Polygon:
    line = LineString([(lon, lat) for lat, lon in points])
    gs = gpd.GeoSeries([line], crs="EPSG:4326")
    proj_crs = _ensure_projected_crs(gs)
    poly_proj = gs.to_crs(proj_crs).buffer(buffer_m).iloc[0]
    poly_wgs84 = gpd.GeoSeries([poly_proj], crs=proj_crs).to_crs("EPSG:4326").iloc[0]
    return poly_wgs84


def _add_edge_speeds_and_times(graph: nx.MultiDiGraph) -> nx.MultiDiGraph:
    try:
        graph = ox.routing.add_edge_speeds(graph)
        graph = ox.routing.add_edge_travel_times(graph)
    except AttributeError:
        graph = ox.add_edge_speeds(graph)
        graph = ox.add_edge_travel_times(graph)
    return graph


def _nearest_nodes(graph: nx.MultiDiGraph, points: Sequence[Tuple[float, float]]) -> List[int]:
    try:
        node_ids: List[int] = []
        for lat, lon in points:
            node = ox.distance.nearest_nodes(graph, X=lon, Y=lat)
            node_ids.append(int(node))
        return node_ids
    except ImportError:
        # Fallback path if optional nearest-neighbor deps (e.g., scikit-learn)
        # are missing in the runtime environment.
        nodes_gdf = ox.graph_to_gdfs(graph, nodes=True, edges=False)
        if nodes_gdf.empty:
            raise RuntimeError("Graph has no nodes for nearest-node snapping.")
        if nodes_gdf.crs is None:
            graph_crs = graph.graph.get("crs", "EPSG:4326")
            nodes_gdf = nodes_gdf.set_crs(graph_crs)

        input_points = gpd.GeoSeries([Point(lon, lat) for lat, lon in points], crs="EPSG:4326")
        proj_crs = _ensure_projected_crs(input_points)
        nodes_proj = nodes_gdf.to_crs(proj_crs)
        points_proj = input_points.to_crs(proj_crs)

        node_ids = []
        for pt in points_proj:
            nearest_node = nodes_proj.geometry.distance(pt).idxmin()
            node_ids.append(int(nearest_node))
        return node_ids


def _snap_info_df(
    graph: nx.MultiDiGraph, points: Sequence[Tuple[float, float]], snapped_nodes: Sequence[int]
) -> pd.DataFrame:
    original_points = gpd.GeoSeries([Point(lon, lat) for lat, lon in points], crs="EPSG:4326")
    snapped_points = gpd.GeoSeries(
        [Point(graph.nodes[node]["x"], graph.nodes[node]["y"]) for node in snapped_nodes], crs="EPSG:4326"
    )
    proj_crs = _ensure_projected_crs(original_points)
    snap_dist_m = original_points.to_crs(proj_crs).distance(snapped_points.to_crs(proj_crs))
    rows = []
    for idx, ((lat, lon), node, dist_m) in enumerate(zip(points, snapped_nodes, snap_dist_m)):
        rows.append(
            {
                "input_index": idx,
                "input_lat": lat,
                "input_lon": lon,
                "snapped_node": int(node),
                "snapped_lat": graph.nodes[node]["y"],
                "snapped_lon": graph.nodes[node]["x"],
                "snap_distance_m": float(dist_m),
            }
        )
    return pd.DataFrame(rows)


def _shortest_path(graph: nx.MultiDiGraph, origin: int, destination: int, weight: str) -> List[int]:
    return nx.shortest_path(graph, source=origin, target=destination, weight=weight)


def _route_via_waypoints(
    graph: nx.MultiDiGraph, snapped_nodes: Sequence[int], weight: str
) -> Tuple[List[int], List[List[int]]]:
    full_route: List[int] = []
    leg_paths: List[List[int]] = []
    for i in range(len(snapped_nodes) - 1):
        path = _shortest_path(graph, snapped_nodes[i], snapped_nodes[i + 1], weight=weight)
        leg_paths.append(path)
        if not full_route:
            full_route.extend(path)
        else:
            full_route.extend(path[1:])
    return full_route, leg_paths


def build_route(
    start: Tuple[float, float],
    middle: Optional[Sequence[Tuple[float, float]]],
    end: Tuple[float, float],
    network_type: str = "drive",
    route_weight: str = "travel_time",
    initial_buffer_m: float = 3000.0,
    max_retries: int = 3,
) -> BuildRouteResult:
    """
    Build a drivable route through start -> middle... -> end.
    Automatically expands search area if route-building fails.
    """
    points = [start] + list(middle or []) + [end]
    if len(points) < 2:
        raise ValueError("At least start and end points are required.")

    last_error: Optional[Exception] = None
    for attempt in range(max_retries):
        buffer_m = initial_buffer_m * (2**attempt)
        polygon = _build_search_polygon(points, buffer_m=buffer_m)
        try:
            graph = ox.graph_from_polygon(
                polygon,
                network_type=network_type,
                simplify=True,
                retain_all=True,
                truncate_by_edge=True,
            )
            if graph.number_of_nodes() == 0:
                raise RuntimeError("Downloaded graph is empty.")
            graph = _add_edge_speeds_and_times(graph)

            snapped_nodes = _nearest_nodes(graph, points)
            snapped_info = _snap_info_df(graph, points, snapped_nodes)

            try:
                route_nodes, leg_paths = _route_via_waypoints(graph, snapped_nodes, weight=route_weight)
            except nx.NetworkXNoPath:
                route_nodes, leg_paths = _route_via_waypoints(graph, snapped_nodes, weight="length")

            return BuildRouteResult(
                graph=graph,
                input_points=points,
                snapped_nodes=snapped_nodes,
                snapped_info=snapped_info,
                route_nodes=route_nodes,
                leg_paths=leg_paths,
                search_polygon_wgs84=polygon,
                search_buffer_m=buffer_m,
            )
        except Exception as exc:  # broad catch to retry with larger polygon
            last_error = exc

    raise RuntimeError(
        f"Unable to build route after {max_retries} attempts. "
        f"Last error: {type(last_error).__name__}: {last_error}"
    )


def _choose_edge_key_data(graph: nx.MultiDiGraph, u: int, v: int, weight: str = "length") -> Tuple[int, Dict[str, Any]]:
    edge_dict = graph.get_edge_data(u, v)
    if not edge_dict:
        raise KeyError(f"No edge found from {u} to {v}.")
    best_key = None
    best_data = None
    best_weight = float("inf")
    for key, data in edge_dict.items():
        current = data.get(weight, data.get("length", float("inf")))
        try:
            current_weight = float(current)
        except (TypeError, ValueError):
            current_weight = float("inf")
        if current_weight < best_weight:
            best_weight = current_weight
            best_key = key
            best_data = data
    if best_key is None or best_data is None:
        key = next(iter(edge_dict))
        return key, edge_dict[key]
    return best_key, best_data


def _oriented_geometry(graph: nx.MultiDiGraph, u: int, v: int, geometry: Optional[LineString]) -> LineString:
    if geometry is None:
        return LineString([(graph.nodes[u]["x"], graph.nodes[u]["y"]), (graph.nodes[v]["x"], graph.nodes[v]["y"])])
    coords = list(geometry.coords)
    if len(coords) < 2:
        return LineString([(graph.nodes[u]["x"], graph.nodes[u]["y"]), (graph.nodes[v]["x"], graph.nodes[v]["y"])])
    start = Point(graph.nodes[u]["x"], graph.nodes[u]["y"])
    d_first = Point(coords[0]).distance(start)
    d_last = Point(coords[-1]).distance(start)
    if d_last < d_first:
        return LineString(list(reversed(coords)))
    return geometry


def _segment_bearing_deg(geometry: LineString) -> Optional[float]:
    coords = list(geometry.coords)
    if len(coords) < 2:
        return None
    lon1, lat1 = coords[0]
    lon2, lat2 = coords[-1]
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    delta_lon = math.radians(lon2 - lon1)
    x = math.sin(delta_lon) * math.cos(phi2)
    y = math.cos(phi1) * math.sin(phi2) - math.sin(phi1) * math.cos(phi2) * math.cos(delta_lon)
    if x == 0 and y == 0:
        return None
    bearing = (math.degrees(math.atan2(x, y)) + 360.0) % 360.0
    return bearing


def _bearing_delta_deg(b1: Optional[float], b2: Optional[float]) -> Optional[float]:
    if b1 is None or b2 is None:
        return None
    curvature = abs(float(b2) - float(b1))
    if curvature > 180.0:
        curvature = 360.0 - curvature
    return curvature


def _curvature_class(curvature_deg: Optional[float]) -> Optional[str]:
    if curvature_deg is None:
        return None
    if curvature_deg < 10.0:
        return "straight"
    if curvature_deg <= 45.0:
        return "moderate"
    return "sharp"


def estimate_lanes(row: Dict[str, Any]) -> Tuple[int, str, float]:
    lanes = _extract_int(row.get("lanes"))
    if lanes is not None:
        return lanes, "lanes_osm", 1.0

    highway = row.get("primary_highway")
    speed = row.get("maxspeed_kph")
    oneway = row.get("oneway_bool")

    if speed is not None:
        if speed >= 90:
            return (4 if not oneway else 2), "lanes_inferred_speed_high", 0.8
        if speed >= 60:
            return (2 if not oneway else 1), "lanes_inferred_speed_medium", 0.7
        return 1, "lanes_inferred_speed_low", 0.6

    mapping = {
        "motorway": 4,
        "trunk": 3,
        "primary": 2,
        "secondary": 2,
        "tertiary": 2,
        "residential": 1,
    }

    return mapping.get(highway, 2), "lanes_inferred_highway_fallback", 0.5


def estimate_speed_kph(row: Dict[str, Any]) -> Tuple[float, str, float]:
    speed = row.get("maxspeed_kph")
    if speed is not None:
        return speed, "speed_osm", 1.0

    highway = row.get("primary_highway")
    mapping = {
        "motorway": 110,
        "trunk": 100,
        "primary": 90,
        "secondary": 80,
        "tertiary": 60,
        "residential": 40,
    }
    return float(mapping.get(highway, 50)), "speed_inferred_highway", 0.6


def _haversine_m(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    radius = 6371000.0
    phi1 = math.radians(lat1)
    phi2 = math.radians(lat2)
    dphi = math.radians(lat2 - lat1)
    dlambda = math.radians(lon2 - lon1)
    a = math.sin(dphi / 2) ** 2 + math.cos(phi1) * math.cos(phi2) * math.sin(dlambda / 2) ** 2
    return 2 * radius * math.asin(math.sqrt(a))


def _extract_int(value: Any) -> Optional[int]:
    text = _first_text(value)
    if text is None:
        return None
    match = re.search(r"\d+", text)
    if not match:
        return None
    try:
        return int(match.group(0))
    except ValueError:
        return None


def _coerce_float(value: Any) -> Optional[float]:
    text = _first_text(value)
    if text is None:
        return None
    match = re.search(r"(\d+(\.\d+)?)", text)
    if not match:
        return None
    try:
        return float(match.group(1))
    except ValueError:
        return None


def _parse_maxspeed_kph(value: Any) -> Optional[float]:
    text = _first_text(value)
    if text is None:
        return None
    lower = text.lower().strip()
    if lower in {"none", "signals", "walk", "variable"}:
        return None

    chunk = re.split(r"[;|,]", lower)[0].strip()
    mph = "mph" in chunk
    match = re.search(r"(\d+(\.\d+)?)", chunk)
    if not match:
        return None
    numeric = float(match.group(1))
    return numeric * 1.60934 if mph else numeric


def _parse_oneway(value: Any) -> Optional[bool]:
    if isinstance(value, bool):
        return value
    text = _first_text(value)
    if text is None:
        return None
    lower = text.lower()
    if lower in {"yes", "true", "1", "-1"}:
        return True
    if lower in {"no", "false", "0"}:
        return False
    return None


def _primary_highway(value: Any) -> Optional[str]:
    text = _first_text(value)
    if text is None:
        return None
    return text


def extract_route_edges(
    graph: nx.MultiDiGraph,
    route_nodes: Sequence[int],
    weight_for_parallel_edges: str = "length",
) -> gpd.GeoDataFrame:
    """
    Extract traversed route segments and requested OSM attributes from the graph.
    """
    rows: List[Dict[str, Any]] = []
    for idx in range(len(route_nodes) - 1):
        u = int(route_nodes[idx])
        v = int(route_nodes[idx + 1])
        key, data = _choose_edge_key_data(graph, u, v, weight=weight_for_parallel_edges)
        geom = _oriented_geometry(graph, u, v, data.get("geometry"))
        length_value = _coerce_float(data.get("length"))
        if length_value is None:
            length_value = float(geom.length)

        row: Dict[str, Any] = {
            "segment_id": idx + 1,
            "from_node": u,
            "to_node": v,
            "edge_key": key,
            "geometry": geom,
            "segment_length_m": float(length_value),
            "bearing_deg": _segment_bearing_deg(geom),
            "travel_time_s": _coerce_float(data.get("travel_time")),
        }
        for col in ROUTE_EDGE_COLUMNS:
            row[col] = data.get(col)

        row["primary_highway"] = _primary_highway(row.get("highway"))
        row["maxspeed_kph"] = _parse_maxspeed_kph(row.get("maxspeed"))
        row["oneway_bool"] = _parse_oneway(row.get("oneway"))

        speed, speed_source, speed_conf = estimate_speed_kph(row)
        row["maxspeed_kph"] = speed
        row["speed_inference_source"] = speed_source
        row["speed_confidence"] = float(min(1.0, max(0.0, speed_conf)))

        lanes, lane_source, lane_conf = estimate_lanes(row)
        row["lanes_count"] = lanes
        row["lane_inference_source"] = lane_source
        row["lane_confidence"] = float(min(1.0, max(0.0, lane_conf)))

        notes: List[str] = []
        if lane_source != "lanes_osm":
            notes.append("lanes_inferred")
        if speed_source != "speed_osm":
            notes.append("maxspeed_inferred")
        if row.get("sidewalk") is None:
            notes.append("sidewalk_unknown")
        row["major_interpreted_notes"] = "; ".join(notes)

        selected_tags = {k: _normalize_value(data.get(k)) for k in ROUTE_EDGE_COLUMNS if not _is_null(data.get(k))}
        row["tags"] = json.dumps(selected_tags, ensure_ascii=True, sort_keys=True)
        rows.append(row)

    gdf = gpd.GeoDataFrame(rows, geometry="geometry", crs="EPSG:4326")
    return gdf


def enrich_route_edges_with_curvature(route_edges_gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    enriched = route_edges_gdf.sort_values("segment_id").copy()
    prev_bearing: Optional[float] = None
    curvature_vals: List[Optional[float]] = []
    curvature_classes: List[Optional[str]] = []

    for idx, bearing in enumerate(enriched["bearing_deg"].tolist()):
        b = (float(bearing) if not _is_null(bearing) else None)
        if idx == 0:
            curvature = 0.0
        else:
            curvature = _bearing_delta_deg(prev_bearing, b)
            if curvature is None:
                curvature = 0.0
        curvature_vals.append(curvature)
        curvature_classes.append(_curvature_class(curvature))
        prev_bearing = b

    enriched["curvature_deg"] = curvature_vals
    enriched["curvature_class"] = curvature_classes
    return gpd.GeoDataFrame(enriched, geometry="geometry", crs=route_edges_gdf.crs or "EPSG:4326")


def get_route_geometry(route_edges_gdf: gpd.GeoDataFrame) -> LineString:
    """
    Combine ordered segment geometries into one route polyline.
    """
    if route_edges_gdf.empty:
        raise ValueError("Route edges are empty; cannot derive route geometry.")

    coord_sequence: List[Tuple[float, float]] = []
    for geom in route_edges_gdf.sort_values("segment_id").geometry:
        if geom is None:
            continue
        coords = list(geom.coords)
        if len(coords) < 2:
            continue
        if not coord_sequence:
            coord_sequence.extend(coords)
            continue
        if coord_sequence[-1] == coords[0]:
            coord_sequence.extend(coords[1:])
        elif coord_sequence[-1] == coords[-1]:
            coord_sequence.extend(list(reversed(coords[:-1])))
        else:
            coord_sequence.extend(coords)

    if len(coord_sequence) < 2:
        raise ValueError("Could not build route geometry from segment coordinates.")
    return LineString(coord_sequence)


def _circular_mean_deg(values: Sequence[float]) -> Optional[float]:
    values = [v for v in values if v is not None]
    if not values:
        return None
    x = sum(math.cos(math.radians(v)) for v in values) / len(values)
    y = sum(math.sin(math.radians(v)) for v in values) / len(values)
    mean = math.degrees(math.atan2(y, x)) % 360.0
    return mean


def _leg_distance_time(
    graph: nx.MultiDiGraph, leg_path: Sequence[int], weight_for_parallel_edges: str = "length"
) -> Tuple[float, Optional[float]]:
    length_sum = 0.0
    time_sum = 0.0
    has_time = False
    for i in range(len(leg_path) - 1):
        _, edge = _choose_edge_key_data(graph, int(leg_path[i]), int(leg_path[i + 1]), weight=weight_for_parallel_edges)
        length_sum += float(edge.get("length", 0.0))
        if edge.get("travel_time") is not None:
            has_time = True
            try:
                time_sum += float(edge["travel_time"])
            except (TypeError, ValueError):
                pass
    return length_sum, (time_sum if has_time else None)


def summarize_edge_attributes(
    route_edges_gdf: gpd.GeoDataFrame,
    route_nodes: Sequence[int],
    graph: nx.MultiDiGraph,
    route_line: LineString,
    leg_paths: Sequence[Sequence[int]],
) -> Dict[str, Any]:
    """
    Summarize route geometry/topology and edge-level attributes.
    """
    route_line_gs = gpd.GeoSeries([route_line], crs="EPSG:4326")
    proj_crs = _ensure_projected_crs(route_line_gs)
    edges_proj = route_edges_gdf.to_crs(proj_crs)
    route_line_proj = route_line_gs.to_crs(proj_crs).iloc[0]

    route_length_m = float(route_line_proj.length)
    route_length_km = route_length_m / 1000.0

    unique_nodes = list(dict.fromkeys([int(n) for n in route_nodes]))
    undirected = graph.to_undirected(as_view=True)
    intersections = [n for n in unique_nodes if undirected.degree(n) >= 3]
    intersection_count = len(intersections)
    intersection_density_per_km = intersection_count / route_length_km if route_length_km > 0 else None

    bearings = [float(v) for v in route_edges_gdf["bearing_deg"].dropna().tolist()]
    mean_bearing = _circular_mean_deg(bearings)

    start_lat, start_lon = route_line.coords[0][1], route_line.coords[0][0]
    end_lat, end_lon = route_line.coords[-1][1], route_line.coords[-1][0]
    straight_dist_m = _haversine_m(start_lat, start_lon, end_lat, end_lon)
    curvature_proxy = route_length_m / straight_dist_m if straight_dist_m > 0 else None

    highway_by_length = (
        route_edges_gdf.assign(primary_highway=route_edges_gdf["primary_highway"].fillna("unknown"))
        .groupby("primary_highway")["segment_length_m"]
        .sum()
        .sort_values(ascending=False)
    )
    pct_by_highway = {
        hw: (float(length) / route_length_m if route_length_m > 0 else None) for hw, length in highway_by_length.items()
    }

    speed_df = route_edges_gdf.dropna(subset=["maxspeed_kph"]).copy()
    weighted_avg_speed_kph = None
    modal_speed_kph = None
    if not speed_df.empty:
        weighted_avg_speed_kph = float(
            (speed_df["maxspeed_kph"] * speed_df["segment_length_m"]).sum() / speed_df["segment_length_m"].sum()
        )
        modal_speed_kph = float(speed_df["maxspeed_kph"].round().mode().iloc[0])

    lane_dist = route_edges_gdf["lanes_count"].value_counts(dropna=False).to_dict()
    lane_dist_clean = {("unknown" if _is_null(k) else int(k)): int(v) for k, v in lane_dist.items()}

    oneway_length = float(route_edges_gdf.loc[route_edges_gdf["oneway_bool"] == True, "segment_length_m"].sum())
    twoway_length = float(route_edges_gdf.loc[route_edges_gdf["oneway_bool"] == False, "segment_length_m"].sum())
    unknown_direction_length = route_length_m - oneway_length - twoway_length

    sidewalk_present_len = float(
        route_edges_gdf.loc[
            route_edges_gdf["sidewalk"].notna() & ~route_edges_gdf["sidewalk"].astype(str).str.lower().isin(["no"]),
            "segment_length_m",
        ].sum()
    )
    sidewalk_no_len = float(
        route_edges_gdf.loc[
            route_edges_gdf["sidewalk"].astype(str).str.lower().isin(["no"]),
            "segment_length_m",
        ].sum()
    )
    sidewalk_unknown_len = route_length_m - sidewalk_present_len - sidewalk_no_len

    bike_tag_present = route_edges_gdf.apply(
        lambda r: (
            (not _is_null(r.get("cycleway")))
            or (not _is_null(r.get("cycleway:left")))
            or (not _is_null(r.get("cycleway:right")))
        ),
        axis=1,
    )
    bike_len = float(route_edges_gdf.loc[bike_tag_present, "segment_length_m"].sum())

    roundabout_edges_count = int(
        route_edges_gdf["junction"].apply(lambda v: str(_first_text(v)).lower() == "roundabout").sum()
    )

    leg_summaries: List[Dict[str, Any]] = []
    for i, leg in enumerate(leg_paths):
        leg_length_m, leg_time_s = _leg_distance_time(graph, leg)
        leg_summaries.append(
            {
                "leg_index": i,
                "from_input_index": i,
                "to_input_index": i + 1,
                "length_m": leg_length_m,
                "length_km": leg_length_m / 1000.0,
                "travel_time_s": leg_time_s,
                "travel_time_min": (leg_time_s / 60.0 if leg_time_s is not None else None),
            }
        )

    return {
        "projected_crs": proj_crs,
        "route_geometry_topology": {
            "route_length_m": route_length_m,
            "route_length_km": route_length_km,
            "route_polyline_wkt": route_line.wkt,
            "segment_count": int(len(route_edges_gdf)),
            "route_node_count": int(len(unique_nodes)),
            "intersection_count": intersection_count,
            "intersection_density_per_km": intersection_density_per_km,
            "mean_bearing_deg": mean_bearing,
            "curvature_proxy_sinuosity": curvature_proxy,
            "leg_summaries": leg_summaries,
        },
        "edge_attribute_summary": {
            "percent_length_by_highway_type": pct_by_highway,
            "weighted_avg_speed_limit_kph": weighted_avg_speed_kph,
            "modal_speed_limit_kph": modal_speed_kph,
            "lane_count_distribution": lane_dist_clean,
            "oneway_proportion_by_length": {
                "oneway": (oneway_length / route_length_m if route_length_m > 0 else None),
                "twoway": (twoway_length / route_length_m if route_length_m > 0 else None),
                "unknown": (unknown_direction_length / route_length_m if route_length_m > 0 else None),
            },
            "sidewalk_availability_by_length": {
                "present": (sidewalk_present_len / route_length_m if route_length_m > 0 else None),
                "explicit_no": (sidewalk_no_len / route_length_m if route_length_m > 0 else None),
                "unknown": (sidewalk_unknown_len / route_length_m if route_length_m > 0 else None),
            },
            "bike_infrastructure_by_length": {
                "has_cycleway_tag": (bike_len / route_length_m if route_length_m > 0 else None),
                "no_cycleway_tag": ((route_length_m - bike_len) / route_length_m if route_length_m > 0 else None),
            },
            "roundabout_edge_count": roundabout_edges_count,
        },
        "route_line_projected_length_m": route_length_m,
        "route_line_projected_wkb_hex": route_line_proj.wkb_hex,
    }


def _features_from_polygon(polygon: Polygon, tags: Dict[str, Any]) -> gpd.GeoDataFrame:
    try:
        return ox.features_from_polygon(polygon, tags=tags)
    except AttributeError:
        return ox.geometries_from_polygon(polygon, tags=tags)


def _classify_feature(row: pd.Series) -> Tuple[str, str]:
    highway = _first_text(row.get("highway"))
    railway = _first_text(row.get("railway"))
    junction = _first_text(row.get("junction"))
    amenity = _first_text(row.get("amenity"))
    public_transport = _first_text(row.get("public_transport"))
    traffic_calming = _first_text(row.get("traffic_calming"))
    barrier = _first_text(row.get("barrier"))
    landuse = _first_text(row.get("landuse"))
    building = _first_text(row.get("building"))
    healthcare = _first_text(row.get("healthcare"))
    parking = _first_text(row.get("parking"))
    cycleway = _first_text(row.get("cycleway"))
    cycleway_left = _first_text(row.get("cycleway:left"))
    cycleway_right = _first_text(row.get("cycleway:right"))
    crossing = _first_text(row.get("crossing"))

    if highway == "traffic_signals":
        return "traffic_signal", "traffic_control"
    if highway == "stop":
        return "stop_sign", "traffic_control"
    if highway == "give_way":
        return "give_way", "traffic_control"
    if highway == "crossing" or crossing is not None or railway in {"level_crossing", "crossing"}:
        return "crossing", "traffic_control"
    if traffic_calming is not None:
        return "traffic_calming", "traffic_control"
    if junction == "roundabout" or highway == "mini_roundabout":
        return "roundabout", "traffic_control"
    if barrier is not None:
        return "barrier", "traffic_control"
    if highway == "bus_stop" or public_transport in {"platform", "stop_position", "station"}:
        return "transit_stop", "transport_context"
    if highway == "cycleway" or cycleway is not None or cycleway_left is not None or cycleway_right is not None:
        return "bike_lane", "transport_context"
    if amenity == "school" or building == "school":
        return "school", "transport_context"
    if amenity == "hospital" or healthcare == "hospital":
        return "hospital", "transport_context"
    if amenity == "parking" or parking is not None:
        return "parking", "transport_context"
    if landuse is not None:
        return "landuse", "landuse_context"
    if building is not None:
        return "building", "landuse_context"
    return "other_context", "other"


def query_corridor_features(
    route_line: LineString,
    corridor_width_m: float,
    tags: Optional[Dict[str, Any]] = None,
) -> Tuple[gpd.GeoDataFrame, Polygon, Polygon, str]:
    """
    Query OSM features that intersect a metric buffer around the route.

    Returns:
      - features GeoDataFrame in EPSG:4326
      - corridor polygon in EPSG:4326
      - corridor polygon in projected CRS
      - projected CRS string
    """
    tags = tags or DEFAULT_CORRIDOR_TAGS
    route_gs = gpd.GeoSeries([route_line], crs="EPSG:4326")
    proj_crs = _ensure_projected_crs(route_gs)
    route_proj = route_gs.to_crs(proj_crs).iloc[0]
    corridor_proj = route_proj.buffer(corridor_width_m)
    corridor_wgs84 = gpd.GeoSeries([corridor_proj], crs=proj_crs).to_crs("EPSG:4326").iloc[0]

    try:
        features = _features_from_polygon(corridor_wgs84, tags=tags)
        if features.empty:
            features = gpd.GeoDataFrame(geometry=[], crs="EPSG:4326")
        else:
            features = features.reset_index(drop=False)
            if features.crs is None:
                features = features.set_crs("EPSG:4326")
            else:
                features = features.to_crs("EPSG:4326")
            features["feature_type"], features["feature_group"] = zip(
                *features.apply(_classify_feature, axis=1)
            )
    except Exception:
        features = gpd.GeoDataFrame(
            columns=["feature_type", "feature_group", "geometry"],
            geometry="geometry",
            crs="EPSG:4326",
        )

    return features, corridor_wgs84, corridor_proj, proj_crs


def _line_fraction_near_features(
    route_line_proj: LineString, features_proj: gpd.GeoDataFrame, feature_type: str, distance_m: float = 100.0
) -> Optional[float]:
    subset = features_proj.loc[features_proj["feature_type"] == feature_type]
    if subset.empty:
        return 0.0
    target = subset.geometry.buffer(distance_m).unary_union
    if target.is_empty or route_line_proj.length <= 0:
        return 0.0
    intersect_len = route_line_proj.intersection(target).length
    return float(intersect_len / route_line_proj.length)


def summarize_corridor_features(
    corridor_features_gdf: gpd.GeoDataFrame,
    corridor_polygon_proj: Polygon,
    route_line: LineString,
    projected_crs: str,
) -> Dict[str, Any]:
    """
    Summarize nearby safety/control/context features in the route corridor.
    """
    route_line_proj = gpd.GeoSeries([route_line], crs="EPSG:4326").to_crs(projected_crs).iloc[0]
    if corridor_features_gdf.empty:
        return {
            "traffic_control_counts": {
                "traffic_signals": 0,
                "stop_controlled": 0,
                "crossings": 0,
                "traffic_calming": 0,
                "roundabouts": 0,
                "barriers": 0,
            },
            "context_counts": {
                "buildings": 0,
                "schools": 0,
                "hospitals": 0,
                "parking": 0,
                "transit_stops": 0,
            },
            "landuse_summary": {},
            "route_proximity": {"fraction_route_within_100m_of_schools": 0.0, "fraction_route_within_100m_of_hospitals": 0.0},
        }

    features_proj = corridor_features_gdf.to_crs(projected_crs)
    features_proj = features_proj.loc[features_proj.geometry.intersects(corridor_polygon_proj)].copy()

    counts = features_proj["feature_type"].value_counts().to_dict()
    traffic_signals = int(counts.get("traffic_signal", 0))
    stop_controlled = int(counts.get("stop_sign", 0) + counts.get("give_way", 0))
    crossings = int(counts.get("crossing", 0))
    calming = int(counts.get("traffic_calming", 0))
    roundabouts = int(counts.get("roundabout", 0))
    barriers = int(counts.get("barrier", 0))

    buildings = int(counts.get("building", 0))
    schools = int(counts.get("school", 0))
    hospitals = int(counts.get("hospital", 0))
    parking = int(counts.get("parking", 0))
    transit_stops = int(counts.get("transit_stop", 0))

    landuse_summary: Dict[str, Dict[str, float]] = {}
    landuse_rows = features_proj.loc[features_proj["feature_type"] == "landuse"].copy()
    if not landuse_rows.empty and "landuse" in landuse_rows.columns:
        landuse_rows = landuse_rows.loc[
            landuse_rows.geometry.geom_type.isin(["Polygon", "MultiPolygon"])
        ].copy()
        if not landuse_rows.empty:
            intersection_area = landuse_rows.geometry.intersection(corridor_polygon_proj).area
            landuse_values = landuse_rows["landuse"].apply(lambda x: _first_text(x) or "unknown")
            area_df = pd.DataFrame({"landuse": landuse_values, "area_sqm": intersection_area})
            area_sum = area_df.groupby("landuse")["area_sqm"].sum().sort_values(ascending=False)
            total_landuse_area = float(area_sum.sum())
            for lu, area in area_sum.items():
                landuse_summary[str(lu)] = {
                    "area_sqm": float(area),
                    "fraction_of_landuse_area": (float(area) / total_landuse_area if total_landuse_area > 0 else None),
                }

    fraction_near_schools = _line_fraction_near_features(route_line_proj, features_proj, "school", distance_m=100.0)
    fraction_near_hospitals = _line_fraction_near_features(route_line_proj, features_proj, "hospital", distance_m=100.0)

    return {
        "traffic_control_counts": {
            "traffic_signals": traffic_signals,
            "stop_controlled": stop_controlled,
            "crossings": crossings,
            "traffic_calming": calming,
            "roundabouts": roundabouts,
            "barriers": barriers,
        },
        "context_counts": {
            "buildings": buildings,
            "schools": schools,
            "hospitals": hospitals,
            "parking": parking,
            "transit_stops": transit_stops,
        },
        "landuse_summary": landuse_summary,
        "route_proximity": {
            "fraction_route_within_100m_of_schools": fraction_near_schools,
            "fraction_route_within_100m_of_hospitals": fraction_near_hospitals,
        },
    }


def query_turn_restrictions_overpass(
    corridor_polygon_wgs84: Polygon,
    overpass_url: str = "https://overpass-api.de/api/interpreter",
    timeout_s: int = 120,
) -> Dict[str, Any]:
    """
    Optional: query turn-restriction relations from Overpass API.
    """
    minx, miny, maxx, maxy = corridor_polygon_wgs84.bounds
    query = f"""
    [out:json][timeout:{timeout_s}];
    (
      relation["type"="restriction"]({miny},{minx},{maxy},{maxx});
    );
    out tags center;
    """
    try:
        response = requests.post(overpass_url, data={"data": query}, timeout=timeout_s)
        response.raise_for_status()
        payload = response.json()
        elements = payload.get("elements", [])
        by_restriction = {}
        for elem in elements:
            tags = elem.get("tags", {})
            restriction = tags.get("restriction", "unknown")
            by_restriction[restriction] = by_restriction.get(restriction, 0) + 1
        return {
            "status": "ok",
            "count": len(elements),
            "by_restriction": by_restriction,
        }
    except Exception as exc:
        return {
            "status": "failed",
            "count": None,
            "error": f"{type(exc).__name__}: {exc}",
            "by_restriction": {},
        }


def _jsonify_records(value: Any) -> Any:
    import math

    if isinstance(value, float) and math.isnan(value):
        return None
    if isinstance(value, dict):
        return {str(k): _jsonify_records(v) for k, v in value.items()}
    if isinstance(value, list):
        return [_jsonify_records(v) for v in value]
    if isinstance(value, tuple):
        return [_jsonify_records(v) for v in value]
    if hasattr(value, "item"):
        v = value.item()
        if isinstance(v, float) and math.isnan(v):
            return None
        return v
    return value


def _sanitize_gdf_for_export(gdf: gpd.GeoDataFrame) -> gpd.GeoDataFrame:
    clean = gdf.copy()
    for col in clean.columns:
        if col == clean.geometry.name:
            continue
        clean[col] = clean[col].apply(
            lambda x: json.dumps(_jsonify_records(x), ensure_ascii=True, sort_keys=True)
            if isinstance(x, (dict, list, tuple, set))
            else x
        )
    return clean


def _slugify_route_name(route_name: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_-]+", "_", route_name.strip())
    safe = re.sub(r"_+", "_", safe).strip("_")
    return safe or "route"


def _point_record(lat: float, lon: float, kind: str) -> Dict[str, Any]:
    return {"latitude": float(lat), "longitude": float(lon), "type": kind}


def _point_from_geometry(geometry: Any) -> Optional[Point]:
    if geometry is None:
        return None
    if geometry.geom_type == "Point":
        return geometry
    try:
        return geometry.representative_point()
    except Exception:
        return None


def extract_points_of_interest(
    route_nodes: Sequence[int],
    graph: nx.MultiDiGraph,
    corridor_features_gdf: gpd.GeoDataFrame,
    route_edges_gdf: gpd.GeoDataFrame,
) -> Dict[str, List[Dict[str, Any]]]:
    intersections: List[Dict[str, Any]] = []
    traffic_signals: List[Dict[str, Any]] = []
    stop_signs: List[Dict[str, Any]] = []
    crossings: List[Dict[str, Any]] = []
    bike_lanes: List[Dict[str, Any]] = []

    seen: Dict[str, set] = {
        "intersections": set(),
        "traffic_signals": set(),
        "stop_signs": set(),
        "crossings": set(),
        "bike_lanes": set(),
    }

    undirected = graph.to_undirected(as_view=True)
    unique_nodes = list(dict.fromkeys([int(n) for n in route_nodes]))
    for node in unique_nodes:
        if undirected.degree(node) < 3:
            continue
        lat = graph.nodes[node]["y"]
        lon = graph.nodes[node]["x"]
        key = (round(lat, 7), round(lon, 7))
        if key in seen["intersections"]:
            continue
        seen["intersections"].add(key)
        intersections.append(_point_record(lat, lon, "intersection"))

    feature_map = {
        "traffic_signal": ("traffic_signals", traffic_signals, "traffic_signal"),
        "stop_sign": ("stop_signs", stop_signs, "stop_sign"),
        "crossing": ("crossings", crossings, "crossing"),
        "bike_lane": ("bike_lanes", bike_lanes, "bike_lane"),
    }
    if not corridor_features_gdf.empty:
        for _, row in corridor_features_gdf.iterrows():
            feature_type = str(row.get("feature_type"))
            if feature_type not in feature_map:
                continue
            bucket_name, bucket, label = feature_map[feature_type]
            point_geom = _point_from_geometry(row.geometry)
            if point_geom is None:
                continue
            key = (round(point_geom.y, 7), round(point_geom.x, 7))
            if key in seen[bucket_name]:
                continue
            seen[bucket_name].add(key)
            bucket.append(_point_record(point_geom.y, point_geom.x, label))

    bike_tag_present = route_edges_gdf.apply(
        lambda r: (
            (not _is_null(r.get("cycleway")))
            or (not _is_null(r.get("cycleway:left")))
            or (not _is_null(r.get("cycleway:right")))
        ),
        axis=1,
    )
    for _, row in route_edges_gdf.loc[bike_tag_present].iterrows():
        pt = _point_from_geometry(row.geometry)
        if pt is None:
            continue
        key = (round(pt.y, 7), round(pt.x, 7))
        if key in seen["bike_lanes"]:
            continue
        seen["bike_lanes"].add(key)
        bike_lanes.append(_point_record(pt.y, pt.x, "bike_lane"))

    return {
        "intersections": intersections,
        "traffic_signals": traffic_signals,
        "stop_signs": stop_signs,
        "crossings": crossings,
        "bike_lanes": bike_lanes,
    }


def build_route_metrics(
    route_edges_gdf: gpd.GeoDataFrame,
    route_geometry_topology: Dict[str, Any],
) -> Dict[str, Any]:
    length_km = float(route_geometry_topology.get("route_length_km", 0.0))
    length_miles = length_km * 0.621371

    total_seconds: Optional[float] = None
    if "travel_time_s" in route_edges_gdf.columns:
        valid = route_edges_gdf["travel_time_s"].dropna()
        if not valid.empty:
            total_seconds = float(valid.astype(float).sum())

    travel_time_min = (total_seconds / 60.0) if total_seconds is not None else None
    return {
        "length_km": length_km,
        "length_miles": length_miles,
        "travel_time_min": travel_time_min,
    }


def build_edge_attributes(route_edges_gdf: gpd.GeoDataFrame) -> List[Dict[str, Any]]:
    rows: List[Dict[str, Any]] = []
    for _, row in route_edges_gdf.sort_values("segment_id").iterrows():
        oneway_bool = row.get("oneway_bool")
        direction = "oneway" if oneway_bool is True else "twoway"
        bearing_deg = row.get("bearing_deg")
        curvature_deg = row.get("curvature_deg")
        maxspeed_kph = row.get("maxspeed_kph")
        lanes_count = row.get("lanes_count")
        travel_time_s = row.get("travel_time_s")
        lane_confidence = row.get("lane_confidence")
        speed_confidence = row.get("speed_confidence")
        rows.append(
            {
                "segment_id": int(row.get("segment_id")),
                "from_node": int(row.get("from_node")),
                "to_node": int(row.get("to_node")),
                "segment_length_m": float(row.get("segment_length_m", 0.0)),
                "bearing_deg": (None if pd.isna(bearing_deg) else bearing_deg),
                "curvature_deg": (None if pd.isna(curvature_deg) else curvature_deg),
                "curvature_class": row.get("curvature_class"),
                "primary_highway": row.get("primary_highway"),
                "maxspeed_kph": (None if pd.isna(maxspeed_kph) else maxspeed_kph),
                "lanes_count": (None if pd.isna(lanes_count) else lanes_count),
                "travel_time_s": (None if pd.isna(travel_time_s) else travel_time_s),
                "lane_inference_source": row.get("lane_inference_source"),
                "lane_confidence": (None if pd.isna(lane_confidence) else lane_confidence),
                "speed_inference_source": row.get("speed_inference_source"),
                "speed_confidence": (None if pd.isna(speed_confidence) else speed_confidence),
                "oneway_bool": oneway_bool,
                "direction": direction,
                "major_interpreted_notes": row.get("major_interpreted_notes"),
            }
        )
    return rows


def build_curvature_analysis(route_edges_gdf: gpd.GeoDataFrame) -> List[Dict[str, Any]]:
    analysis: List[Dict[str, Any]] = []
    for _, row in route_edges_gdf.sort_values("segment_id").iterrows():
        bearing_deg = row.get("bearing_deg")
        curvature_deg = row.get("curvature_deg")
        analysis.append(
            {
                "segment_id": int(row.get("segment_id")),
                "bearing_deg": (None if pd.isna(bearing_deg) else bearing_deg),
                "curvature_deg": (None if pd.isna(curvature_deg) else curvature_deg),
                "curvature_class": row.get("curvature_class"),
            }
        )
    return analysis


def _export_optional_map(
    route_line: LineString,
    points_of_interest: Dict[str, List[Dict[str, Any]]],
    output_html_path: Path,
    input_points: Sequence[Tuple[float, float]],
) -> bool:
    try:
        import folium  # optional dependency
    except ImportError:
        return False

    center_lat, center_lon = input_points[0]
    fmap = folium.Map(location=[center_lat, center_lon], zoom_start=13, tiles="cartodbpositron")

    folium.GeoJson(
        gpd.GeoSeries([route_line], crs="EPSG:4326").to_json(),
        name="Route",
        style_function=lambda _: {"color": "#1565C0", "weight": 5, "opacity": 0.9},
    ).add_to(fmap)

    for poi_key, poi_color in {
        "traffic_signals": "#D32F2F",
        "stop_signs": "#EF6C00",
        "crossings": "#6A1B9A",
        "bike_lanes": "#2E7D32",
    }.items():
        feature_group = folium.FeatureGroup(name=poi_key)
        for poi in points_of_interest.get(poi_key, []):
            folium.CircleMarker(
                location=[float(poi["latitude"]), float(poi["longitude"])],
                radius=4,
                color=poi_color,
                fill=True,
                fill_opacity=0.9,
            ).add_to(feature_group)
        feature_group.add_to(fmap)

    for i, (lat, lon) in enumerate(input_points):
        label = "Start" if i == 0 else ("End" if i == len(input_points) - 1 else f"Waypoint {i}")
        if i == 0:
            icon_color = "green"
        elif i == len(input_points) - 1:
            icon_color = "red"
        else:
            icon_color = "blue"
        folium.Marker([lat, lon], tooltip=label, icon=folium.Icon(color=icon_color)).add_to(fmap)

    folium.LayerControl(collapsed=False).add_to(fmap)
    fmap.save(str(output_html_path))
    return True


def export_outputs(
    route_output: Dict[str, Any],
    route_line: LineString,
    output_dir: str,
    route_name: str,
    input_points: Sequence[Tuple[float, float]],
    points_of_interest: Dict[str, List[Dict[str, Any]]],
    export_map: bool = True,
) -> Dict[str, str]:
    """
    Export one JSON output and optional map for the selected route.
    """
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    route_slug = _slugify_route_name(route_name)
    summary_path = out_dir / f"{route_slug}.json"
    map_path = out_dir / f"{route_slug}_map.html"

    with open(summary_path, "w", encoding="utf-8") as fp:
        json.dump(_jsonify_records(route_output), fp, ensure_ascii=True, indent=2, sort_keys=True)

    exported_map = False
    if export_map:
        exported_map = _export_optional_map(
            route_line=route_line,
            points_of_interest=points_of_interest,
            output_html_path=map_path,
            input_points=input_points,
        )

    return {
        "route_output_json": str(summary_path),
        "route_map_html": (str(map_path) if exported_map else "not_exported_folium_missing"),
    }


def _human_report(route_output: Dict[str, Any], exported_paths: Dict[str, str]) -> str:
    metrics = route_output["route_metrics"]
    pois = route_output["points_of_interest"]
    corridor = route_output["corridor_summary"]
    control = corridor["traffic_control_counts"]

    return (
        "\nRoute OSM Analysis Report\n"
        "-------------------------\n"
        f"Route: {route_output['route_name']}\n"
        f"Length: {metrics['length_km']:.3f} km ({metrics['length_miles']:.3f} miles)\n"
        f"Estimated travel time: {metrics['travel_time_min']} min\n"
        f"POIs: intersections={len(pois['intersections'])}, "
        f"signals={len(pois['traffic_signals'])}, stops={len(pois['stop_signs'])}, "
        f"crossings={len(pois['crossings'])}, bike_lanes={len(pois['bike_lanes'])}\n"
        f"Traffic controls in corridor: signals={control['traffic_signals']}, "
        f"stop/yield={control['stop_controlled']}, crossings={control['crossings']}, "
        f"traffic calming={control['traffic_calming']}, roundabouts={control['roundabouts']}\n"
        "Outputs:\n"
        f"- route_output.json: {exported_paths['route_output_json']}\n"
        f"- route_map.html: {exported_paths['route_map_html']}\n"
    )


def main(
    start: Tuple[float, float],
    middle: Optional[Sequence[Tuple[float, float]]],
    end: Tuple[float, float],
    corridor_width_m: float = 30.0,
    output_dir: str = "output",
    route_name: str = "Selected Route",
    query_turn_restrictions: bool = True,
    export_map: bool = True,
) -> Dict[str, Any]:
    """
    End-to-end route-aware OSM analysis.
    """
    build_result = build_route(start=start, middle=middle, end=end, network_type="drive")
    route_edges = extract_route_edges(build_result.graph, build_result.route_nodes, weight_for_parallel_edges="length")
    route_edges = enrich_route_edges_with_curvature(route_edges)
    route_line = get_route_geometry(route_edges)

    corridor_features, corridor_wgs84, corridor_proj, proj_crs = query_corridor_features(
        route_line=route_line,
        corridor_width_m=corridor_width_m,
    )

    edge_summary = summarize_edge_attributes(
        route_edges_gdf=route_edges,
        route_nodes=build_result.route_nodes,
        graph=build_result.graph,
        route_line=route_line,
        leg_paths=build_result.leg_paths,
    )

    corridor_summary = summarize_corridor_features(
        corridor_features_gdf=corridor_features,
        corridor_polygon_proj=corridor_proj,
        route_line=route_line,
        projected_crs=proj_crs,
    )

    turn_restrictions = (
        query_turn_restrictions_overpass(corridor_wgs84) if query_turn_restrictions else {"status": "skipped"}
    )

    points_of_interest = extract_points_of_interest(
        route_nodes=build_result.route_nodes,
        graph=build_result.graph,
        corridor_features_gdf=corridor_features,
        route_edges_gdf=route_edges,
    )
    route_metrics = build_route_metrics(route_edges, edge_summary["route_geometry_topology"])
    edge_attributes = build_edge_attributes(route_edges)
    curvature_analysis = build_curvature_analysis(route_edges)

    route_output: Dict[str, Any] = {
        "route_name": route_name,
        "inputs": {
            "start": list(start),
            "middle": [list(p) for p in (middle or [])],
            "end": list(end),
            "corridor_width_m": corridor_width_m,
        },
        "route_metrics": route_metrics,
        "points_of_interest": points_of_interest,
        "edge_attributes": edge_attributes,
        "curvature_analysis": curvature_analysis,
        "corridor_summary": corridor_summary,
        "snap_summary": build_result.snapped_info.to_dict(orient="records"),
        "route_geometry_topology": edge_summary["route_geometry_topology"],
        "edge_attribute_summary": edge_summary["edge_attribute_summary"],
        "turn_restrictions": turn_restrictions,
    }

    export_paths = export_outputs(
        route_output=route_output,
        route_line=route_line,
        output_dir=output_dir,
        route_name=route_name,
        input_points=build_result.input_points,
        points_of_interest=points_of_interest,
        export_map=export_map,
    )

    report = _human_report(route_output, export_paths)
    print(report)

    return {
        "route_summary": route_output,
        "route_output": route_output,
        "route_edges_gdf": route_edges,
        "corridor_features_gdf": corridor_features,
        "export_paths": export_paths,
        "report": report,
    }


if __name__ == "__main__":
    print("Available routes:")
    for key, route in ROUTES.items():
        print(f"{key}: {route['name']}")

    choice = input("Select route (1, 2, or 3): ")
    if choice not in ROUTES:
        raise ValueError("Invalid route selection")

    selected = ROUTES[choice]

    main(
        start=selected["start"],
        middle=selected["waypoints"],
        end=selected["end"],
        corridor_width_m=CORRIDOR_WIDTH_M,
        output_dir=OUTPUT_DIR,
        route_name=selected["name"],
        query_turn_restrictions=True,
        export_map=True,
    )
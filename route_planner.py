"""
Multi-route planner + complexity scorer.

What this script does:
- Loads START/END from a JSON file (default: output/selected_points.json).
- Downloads a drivable OSM network around the points.
- Generates K candidate routes (k-shortest simple paths by length).
- Computes a route "complexity score" for each candidate using:
  - intersection density, sinuosity proxy
  - traffic signal density, crossing density (via corridor query)
- Writes JSON summaries and creates an interactive HTML comparison map.

Typical workflow:
- Run `python dc_point_picker.py` to create output/selected_points.json
- Run `python route_planner.py --k 3 --output-dir output`

Outputs (in --output-dir):
- route_planning_map.html
- routes_comparison.json
- routes_complexity.json

Author: Ahmad Mansour GWU
"""

from __future__ import annotations

import argparse
import json
import math
import webbrowser
from itertools import islice
from pathlib import Path
from typing import Any, Dict, List, Sequence, Tuple

import networkx as nx
import osmnx as ox

import osm_route_analysis as ora
from route_complexity import classify_complexity, compute_complexity_score


LatLon = Tuple[float, float]

# For route planning we only need signals/crossings for scoring + visualization.
PLANNER_CORRIDOR_TAGS: Dict[str, Any] = {
    "highway": ["traffic_signals", "crossing"],
    "railway": ["level_crossing", "crossing"],
}

def _load_selected_points(path: Path) -> Tuple[LatLon, LatLon]:
    if not path.exists():
        raise FileNotFoundError(str(path))
    data = json.loads(path.read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("selected_points.json must be a JSON object")
    try:
        start_raw = data["START"]
        end_raw = data["END"]
        start = (float(start_raw[0]), float(start_raw[1]))
        end = (float(end_raw[0]), float(end_raw[1]))
    except Exception as exc:  # noqa: BLE001
        raise ValueError("selected_points.json must include START and END as [lat, lon]") from exc
    return start, end


def _graph_from_points(start: LatLon, end: LatLon, buffer_m: float) -> nx.MultiDiGraph:
    lat1, lon1 = start
    lat2, lon2 = end
    lat_mid = (lat1 + lat2) / 2.0
    lon_mid = (lon1 + lon2) / 2.0

    # bbox padding in degrees (approx)
    lat_buf = buffer_m / 111_320.0
    lon_buf = buffer_m / (111_320.0 * max(0.1, abs(math.cos(math.radians(lat_mid)))))

    north = max(lat1, lat2) + lat_buf
    south = min(lat1, lat2) - lat_buf
    east = max(lon1, lon2) + lon_buf
    west = min(lon1, lon2) - lon_buf

    # OSMnx has changed graph_from_bbox signature across versions.
    try:
        return ox.graph_from_bbox(
            north=north,
            south=south,
            east=east,
            west=west,
            network_type="drive",
            simplify=True,
        )
    except TypeError:
        return ox.graph_from_bbox(
            # OSMnx 2.x expects bbox=(west, south, east, north)
            bbox=(west, south, east, north),
            network_type="drive",
            simplify=True,
        )


def _to_digraph_min_length(G: nx.MultiDiGraph) -> nx.DiGraph:
    DG = nx.DiGraph()
    DG.add_nodes_from((n, dict(G.nodes[n])) for n in G.nodes)
    for u, v, data in G.edges(data=True):
        w = float(data.get("length", 1.0) or 1.0)
        if DG.has_edge(u, v):
            if w < float(DG[u][v].get("length", w)):
                DG[u][v]["length"] = w
        else:
            DG.add_edge(u, v, length=w)
    return DG


def _k_shortest_routes(
    G: nx.MultiDiGraph, start: LatLon, end: LatLon, k: int
) -> List[List[int]]:
    start_node = ox.distance.nearest_nodes(G, X=[start[1]], Y=[start[0]])[0]
    end_node = ox.distance.nearest_nodes(G, X=[end[1]], Y=[end[0]])[0]

    DG = _to_digraph_min_length(G)
    gen = nx.shortest_simple_paths(DG, source=start_node, target=end_node, weight="length")
    return [list(p) for p in islice(gen, k)]


def _evaluate_route(
    *,
    graph: nx.MultiDiGraph,
    route_nodes: Sequence[int],
    corridor_width_m: float,
) -> Dict[str, Any]:
    route_edges_gdf = ora.extract_route_edges(graph, route_nodes, weight_for_parallel_edges="length")
    route_line = ora.get_route_geometry(route_edges_gdf)
    edge_summary = ora.summarize_edge_attributes(
        route_edges_gdf=route_edges_gdf,
        route_nodes=route_nodes,
        graph=graph,
        route_line=route_line,
        leg_paths=[route_nodes],
    )

    topo = edge_summary["route_geometry_topology"]
    route_length_km = float(topo["route_length_km"])
    intersection_density = float(topo["intersection_density_per_km"] or 0.0)
    sinuosity = float(topo["curvature_proxy_sinuosity"] or 1.0)

    corridor_features = None
    signals = 0
    crossings = 0
    try:
        corridor_features, corridor_wgs84, corridor_proj, proj_crs = ora.query_corridor_features(
            route_line=route_line,
            corridor_width_m=corridor_width_m,
            tags=PLANNER_CORRIDOR_TAGS,
        )
        corridor_summary = ora.summarize_corridor_features(
            corridor_features_gdf=corridor_features,
            corridor_polygon_proj=corridor_proj,
            route_line=route_line,
            projected_crs=proj_crs,
        )
        counts = corridor_summary.get("traffic_control_counts", {})
        signals = int(counts.get("traffic_signals", 0))
        crossings = int(counts.get("crossings", 0))
    except Exception:
        corridor_features = None

    traffic_signal_density = (signals / route_length_km) if route_length_km > 0 else 0.0
    crossing_density = (crossings / route_length_km) if route_length_km > 0 else 0.0

    complexity_score = compute_complexity_score(
        intersection_density=intersection_density,
        sinuosity=sinuosity,
        traffic_signal_density=traffic_signal_density,
        crossing_density=crossing_density,
    )

    return {
        "route_length_km": route_length_km,
        "intersection_density": intersection_density,
        "sinuosity": sinuosity,
        "traffic_signal_density": traffic_signal_density,
        "crossing_density": crossing_density,
        "complexity_score": complexity_score,
        "classification": classify_complexity(complexity_score),
        "route_line_wkt": route_line.wkt,
        "corridor_features": corridor_features,
    }


def _route_nodes_latlon(G: nx.MultiDiGraph, route_nodes: Sequence[int]) -> List[LatLon]:
    pts: List[LatLon] = []
    for n in route_nodes:
        nd = G.nodes[n]
        pts.append((float(nd["y"]), float(nd["x"])))
    return pts


def _make_route_planning_map(
    *,
    output_path: Path,
    start: LatLon,
    end: LatLon,
    route_results: List[Dict[str, Any]],
    best_route_id: str,
) -> None:
    import folium

    center = ((start[0] + end[0]) / 2.0, (start[1] + end[1]) / 2.0)
    fmap = folium.Map(location=[center[0], center[1]], zoom_start=13, tiles="cartodbpositron")

    colors = ["#1E88E5", "#FB8C00", "#8E24AA", "#00897B", "#F4511E"]
    all_bounds: List[LatLon] = [start, end]

    for idx, rr in enumerate(route_results):
        rid = rr["route_id"]
        pts = rr["route_latlon"]
        all_bounds.extend(pts)
        color = colors[idx % len(colors)]
        folium.PolyLine(
            locations=pts,
            color=color,
            weight=5,
            opacity=0.75,
            tooltip=f"Route {rid} score={rr['complexity_score']:.3f}",
        ).add_to(fmap)

    # Highlight best route
    best = next((r for r in route_results if r["route_id"] == best_route_id), None)
    if best is not None:
        folium.PolyLine(
            locations=best["route_latlon"],
            color="green",
            weight=10,
            opacity=0.9,
            tooltip=f"Best route: {best_route_id} (score={best['complexity_score']:.3f})",
        ).add_to(fmap)

        # Add traffic signal / crossing markers for best route corridor
        cfeatures = best.get("_corridor_features_gdf")
        if cfeatures is not None and getattr(cfeatures, "empty", True) is False:
            sig_group = folium.FeatureGroup(name="Traffic signals", show=True)
            cr_group = folium.FeatureGroup(name="Crossings", show=True)
            for _, row in cfeatures.iterrows():
                ftype = str(row.get("feature_type") or "").strip().lower()
                geom = row.geometry
                if geom is None:
                    continue
                if geom.geom_type == "Point":
                    lat = float(geom.y)
                    lon = float(geom.x)
                else:
                    rp = geom.representative_point()
                    lat = float(rp.y)
                    lon = float(rp.x)

                if ftype not in {"traffic_signal", "crossing"}:
                    continue

                popup_html = (
                    f"<b>feature_type</b>: {ftype}<br/>"
                    f"<b>latitude</b>: {lat:.6f}<br/>"
                    f"<b>longitude</b>: {lon:.6f}"
                )
                if ftype == "traffic_signal":
                    folium.Marker(
                        location=[lat, lon],
                        icon=folium.Icon(color="red", icon="info-sign"),
                        popup=folium.Popup(popup_html, max_width=300),
                    ).add_to(sig_group)
                else:
                    folium.Marker(
                        location=[lat, lon],
                        icon=folium.Icon(color="blue", icon="info-sign"),
                        popup=folium.Popup(popup_html, max_width=300),
                    ).add_to(cr_group)

            sig_group.add_to(fmap)
            cr_group.add_to(fmap)

    folium.Marker([start[0], start[1]], tooltip="START").add_to(fmap)
    folium.Marker([end[0], end[1]], tooltip="END").add_to(fmap)

    # Info panel with route scores
    rows = "".join(
        f"<tr><td>Route {r['route_id']}</td><td style='text-align:right;'><b>{r['complexity_score']:.3f}</b></td></tr>"
        for r in route_results
    )
    panel_html = f"""
    <div style="
        position: fixed;
        bottom: 16px;
        left: 16px;
        z-index: 9999;
        background: rgba(255, 255, 255, 0.95);
        border: 1px solid #bbb;
        border-radius: 8px;
        padding: 10px 12px;
        box-shadow: 0 2px 10px rgba(0,0,0,0.15);
        font-family: Arial, sans-serif;
        font-size: 12px;
        min-width: 220px;
    ">
      <div style="font-size: 14px; font-weight: 700; margin-bottom: 6px;">Route Comparison</div>
      <table style="width:100%; border-collapse: collapse;">
        {rows}
      </table>
      <div style="margin-top:8px;"><b>Best route selected</b>: {best_route_id}</div>
    </div>
    """
    fmap.get_root().html.add_child(folium.Element(panel_html))

    if all_bounds:
        fmap.fit_bounds(all_bounds)
    folium.LayerControl(collapsed=False).add_to(fmap)
    fmap.save(str(output_path))


def run(
    *,
    k: int = 3,
    buffer_m: float = 3000.0,
    corridor_width_m: float = 30.0,
    output_dir: str = "output",
    selected_points_path: str = "output/selected_points.json",
    open_browser: bool = True,
) -> Dict[str, Any]:
    out_dir = Path(output_dir)
    out_dir.mkdir(parents=True, exist_ok=True)

    points_path = Path(selected_points_path)
    try:
        start, end = _load_selected_points(points_path)
    except FileNotFoundError:
        print(f"Missing `{points_path}`. Run `python dc_point_picker.py` first to create it.")
        return {"error": "missing_selected_points", "selected_points_path": str(points_path)}

    # Keep network calls bounded: avoid hanging indefinitely.
    try:
        ox.settings.requests_timeout = 60
    except Exception:
        pass

    try:
        G = _graph_from_points(start, end, buffer_m=buffer_m)
    except Exception as exc:
        print(f"Failed to download OSM network. Check your internet/Overpass access. Error: {exc}")
        return {"error": "network_download_failed", "details": str(exc)}
    routes = _k_shortest_routes(G, start, end, k=k)
    if not routes:
        raise RuntimeError("No candidate routes found.")

    route_ids = [chr(ord("A") + i) for i in range(len(routes))]
    evaluated: List[Dict[str, Any]] = []
    for rid, nodes in zip(route_ids, routes):
        metrics = _evaluate_route(graph=G, route_nodes=nodes, corridor_width_m=corridor_width_m)
        evaluated.append(
            {
                "route_id": rid,
                "complexity_score": float(metrics["complexity_score"]),
                "classification": metrics["classification"],
                "route_length_km": float(metrics["route_length_km"]),
                "intersection_density": float(metrics["intersection_density"]),
                "sinuosity": float(metrics["sinuosity"]),
                "traffic_signal_density": float(metrics["traffic_signal_density"]),
                "crossing_density": float(metrics["crossing_density"]),
                "route_latlon": _route_nodes_latlon(G, nodes),
                "_corridor_features_gdf": metrics["corridor_features"],
            }
        )

    best = min(evaluated, key=lambda r: r["complexity_score"])
    best_route_id = str(best["route_id"])

    comparison = {
        "routes": [{"route": r["route_id"], "complexity": r["complexity_score"]} for r in evaluated],
        "best_route": best_route_id,
    }
    comparison_path = out_dir / "routes_comparison.json"
    comparison_path.write_text(json.dumps(comparison, indent=2), encoding="utf-8")

    # Full per-route complexity metrics (A/B/C) for downstream use
    routes_complexity = {
        "routes": [
            {
                "route": r["route_id"],
                "route_length_km": r["route_length_km"],
                "intersection_density": r["intersection_density"],
                "sinuosity": r["sinuosity"],
                "traffic_signal_density": r["traffic_signal_density"],
                "crossing_density": r["crossing_density"],
                "complexity_score": r["complexity_score"],
                "classification": r["classification"],
            }
            for r in evaluated
        ],
        "best_route": best_route_id,
    }
    (out_dir / "routes_complexity.json").write_text(
        json.dumps(routes_complexity, indent=2), encoding="utf-8"
    )

    map_path = out_dir / "route_planning_map.html"
    _make_route_planning_map(
        output_path=map_path,
        start=start,
        end=end,
        route_results=evaluated,
        best_route_id=best_route_id,
    )

    if open_browser:
        try:
            webbrowser.open(map_path.resolve().as_uri(), new=2)
        except Exception:
            webbrowser.open(str(map_path), new=2)

    # Remove non-serializable items before returning
    for r in evaluated:
        r.pop("_corridor_features_gdf", None)
    return {"selection": {"start": start, "end": end}, "comparison": comparison, "routes": evaluated}


def main() -> int:
    parser = argparse.ArgumentParser(description="AV route planning orchestrator (multi-route + complexity).")
    parser.add_argument("--k", type=int, default=3, help="Number of candidate routes (default: 3).")
    parser.add_argument("--buffer-m", type=float, default=3000.0, help="OSM network bbox padding (meters).")
    parser.add_argument("--corridor-width-m", type=float, default=30.0, help="Corridor width (meters).")
    parser.add_argument("--output-dir", default="output", help="Output directory (default: output).")
    parser.add_argument(
        "--selected-points",
        default="output/selected_points.json",
        help="Path to selected points JSON (default: output/selected_points.json).",
    )
    parser.add_argument(
        "--no-open",
        action="store_true",
        help="Do not open route_planning_map.html in browser.",
    )
    args = parser.parse_args()

    result = run(
        k=args.k,
        buffer_m=args.buffer_m,
        corridor_width_m=args.corridor_width_m,
        output_dir=args.output_dir,
        selected_points_path=args.selected_points,
        open_browser=(not args.no_open),
    )
    if "error" not in result:
        print(f"Wrote {Path(args.output_dir) / 'route_planning_map.html'}")
        print(f"Wrote {Path(args.output_dir) / 'routes_comparison.json'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


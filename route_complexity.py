"""
Route complexity report generator.

What this script does:
- Reads `route_summary.json` (produced by `osm_route_analysis.py`) and extracts:
  - route length (km)
  - intersection density (per km)
  - sinuosity proxy
- Computes traffic signal & crossing densities using either:
  - `corridor_features.csv` (preferred), or
  - `route_summary.json` fallback counts
- Writes `route_complexity.json` with the final score + classification.

How to use:
- Run: `python route_complexity.py --results-dir output`

Author: Ahmad Mansour GWU
"""

import argparse
import csv
import json
from pathlib import Path
from typing import Any, Dict, Tuple


def compute_complexity_score(
    *,
    intersection_density: float,
    sinuosity: float,
    traffic_signal_density: float,
    crossing_density: float,
) -> float:
    return (
        0.35 * float(intersection_density)
        + 0.25 * float(sinuosity)
        + 0.20 * float(traffic_signal_density)
        + 0.20 * float(crossing_density)
    )


def classify_complexity(score: float) -> str:
    if score < 30:
        return "Easy"
    if score < 60:
        return "Moderate"
    return "Complex"


def _pick_results_dir(preferred: Path) -> Path:
    """
    Prefer the user-provided directory, but fall back to `output/` if the repo is
    using osm_route_analysis.py defaults and `results/` doesn't exist yet.
    """
    preferred_summary = preferred / "route_summary.json"
    if preferred_summary.exists():
        return preferred

    fallback = Path("output")
    fallback_summary = fallback / "route_summary.json"
    if preferred.name == "results" and fallback_summary.exists():
        return fallback

    return preferred


def _load_json(path: Path) -> Dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        data = json.load(f)
    if not isinstance(data, dict):
        raise ValueError(f"Expected JSON object in {path}, got {type(data).__name__}")
    return data


def _extract_summary_metrics(route_summary: Dict[str, Any]) -> Tuple[float, float, float]:
    # Preferred location (as produced by osm_route_analysis.py)
    topo = route_summary.get("route_geometry_topology")
    if isinstance(topo, dict):
        route_length_km = topo.get("route_length_km")
        intersection_density = topo.get("intersection_density_per_km")
        sinuosity = topo.get("curvature_proxy_sinuosity")
    else:
        route_length_km = route_summary.get("route_length_km")
        intersection_density = route_summary.get("intersection_density")
        sinuosity = route_summary.get("sinuosity")

    if route_length_km is None or intersection_density is None or sinuosity is None:
        raise KeyError(
            "Missing one or more required fields in route_summary.json. "
            "Expected route_length_km, intersection_density(_per_km), sinuosity(curvature_proxy_sinuosity)."
        )

    return float(route_length_km), float(intersection_density), float(sinuosity)


def _count_controls_from_corridor_csv(path: Path) -> Tuple[int, int]:
    """
    Count traffic signals and crossings from corridor_features.csv.

    If a `feature_type` column exists (as produced by osm_route_analysis.py's classifier),
    it is used. Otherwise, fall back to searching the row text.
    """
    signals = 0
    crossings = 0

    with path.open("r", encoding="utf-8", newline="") as f:
        reader = csv.DictReader(f)
        if reader.fieldnames is None:
            return 0, 0

        fieldnames_lower = {h.lower(): h for h in reader.fieldnames if h is not None}
        feature_type_key = fieldnames_lower.get("feature_type")

        for row in reader:
            if not isinstance(row, dict):
                continue

            if feature_type_key and row.get(feature_type_key) is not None:
                ft = str(row.get(feature_type_key)).strip().lower()
                if ft == "traffic_signal":
                    signals += 1
                    continue
                if ft == "crossing":
                    crossings += 1
                    continue

            # Fallback: scan all values for OSM-like tokens
            values_text = " ".join(str(v) for v in row.values() if v is not None).lower()
            if "traffic_signals" in values_text or "traffic signal" in values_text:
                signals += 1
            if "crossing" in values_text:
                crossings += 1

    return signals, crossings


def _count_controls_from_route_summary(route_summary: Dict[str, Any]) -> Tuple[int, int]:
    cfs = route_summary.get("corridor_feature_summary")
    if not isinstance(cfs, dict):
        return 0, 0
    tcc = cfs.get("traffic_control_counts")
    if not isinstance(tcc, dict):
        return 0, 0

    signals = int(tcc.get("traffic_signals", 0) or 0)
    crossings = int(tcc.get("crossings", 0) or 0)
    return signals, crossings


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compute route complexity from osm_route_analysis outputs."
    )
    parser.add_argument(
        "--results-dir",
        default="results",
        help="Directory containing route_summary.json and corridor_features.csv (default: results).",
    )
    args = parser.parse_args()

    requested_dir = Path(args.results_dir)
    base_dir = _pick_results_dir(requested_dir)

    summary_path = base_dir / "route_summary.json"
    corridor_csv_path = base_dir / "corridor_features.csv"
    output_path = base_dir / "route_complexity.json"

    if not summary_path.exists():
        raise FileNotFoundError(f"Missing {summary_path}")

    route_summary = _load_json(summary_path)
    route_length_km, intersection_density, sinuosity = _extract_summary_metrics(route_summary)

    if route_length_km <= 0:
        raise ValueError(f"route_length_km must be > 0, got {route_length_km}")

    if corridor_csv_path.exists():
        signals, crossings = _count_controls_from_corridor_csv(corridor_csv_path)
    else:
        signals, crossings = _count_controls_from_route_summary(route_summary)
    traffic_signal_density = signals / route_length_km
    crossing_density = crossings / route_length_km

    complexity_score = compute_complexity_score(
        intersection_density=intersection_density,
        sinuosity=sinuosity,
        traffic_signal_density=traffic_signal_density,
        crossing_density=crossing_density,
    )
    classification = classify_complexity(complexity_score)

    base_dir.mkdir(parents=True, exist_ok=True)
    result: Dict[str, Any] = {
        "route_length_km": route_length_km,
        "intersection_density": intersection_density,
        "sinuosity": sinuosity,
        "traffic_signal_density": traffic_signal_density,
        "crossing_density": crossing_density,
        "complexity_score": complexity_score,
        "classification": classification,
    }

    with output_path.open("w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2, sort_keys=True)

    print(f"Wrote {output_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())


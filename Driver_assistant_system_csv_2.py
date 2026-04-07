import pandas as pd
import tkinter as tk
import time
import json
from shapely import wkt
from shapely.geometry import Point
import geopandas as gpd


M_TO_MILES = 0.000621371
KM_TO_MILES = 0.621371
KPH_TO_MPH = 0.621371


def meters_to_miles(meters):
    return meters * M_TO_MILES


def km_to_miles(km):
    return km * KM_TO_MILES


def kph_to_mph(kph):
    return kph * KPH_TO_MPH


def get_symbols(
    curvature_state,
    intersection_warning,
    control_on_segment,
    road_direction,
    density_class
):
    symbols = []

    if curvature_state == "High Curvature":
        symbols.append("↷")
    elif curvature_state == "Moderate Curve":
        symbols.append("↱")
    else:
        symbols.append("→")

    if intersection_warning is not None:
        if "Signalized" in intersection_warning:
            symbols.append("[SIG]")
        elif "Stop" in intersection_warning:
            symbols.append("[STOP]")
        else:
            symbols.append("[INT]")

    if control_on_segment == "Traffic signal on current segment":
        symbols.append("[SIG]")
    elif control_on_segment == "Stop-controlled intersection on current segment":
        symbols.append("[STOP]")

    if road_direction == "One-way":
        symbols.append("➡")
    else:
        symbols.append("⇄")

    if density_class == "High":
        symbols.append("≋")
    elif density_class == "Medium":
        symbols.append("≈")
    else:
        symbols.append("—")

    return "  ".join(symbols)


def prepare_segment_geometries(edge_attributes):
    rows = []

    for segment in edge_attributes:
        geom_wkt = segment.get("segment_geometry_wkt")
        if geom_wkt is None:
            continue

        geom = wkt.loads(geom_wkt)

        rows.append({
            "segment_id": segment.get("segment_id"),
            "segment_start_m": segment.get("segment_start_m"),
            "segment_end_m": segment.get("segment_end_m"),
            "segment_length_m": segment.get("segment_length_m"),
            "geometry": geom
        })

    segments_gdf = gpd.GeoDataFrame(rows, geometry="geometry", crs="EPSG:4326")

    projected_crs = segments_gdf.estimate_utm_crs()
    if projected_crs is None:
        projected_crs = "EPSG:3857"

    segments_gdf = segments_gdf.to_crs(projected_crs)

    return segments_gdf, projected_crs


def locate_vehicle_on_route_segment(vehicle_x, vehicle_y, segments_gdf):
    vehicle_point = Point(vehicle_x, vehicle_y)

    best_match = None
    best_distance = float("inf")

    for _, row in segments_gdf.iterrows():
        line = row.geometry
        lateral_distance = vehicle_point.distance(line)

        if lateral_distance < best_distance:
            best_distance = lateral_distance

            distance_along_segment_m = line.project(vehicle_point)
            snapped_point = line.interpolate(distance_along_segment_m)

            distance_along_route_m = float(row["segment_start_m"]) + float(distance_along_segment_m)

            best_match = {
                "segment_id": row["segment_id"],
                "distance_to_segment_m": float(lateral_distance),
                "distance_along_segment_m": float(distance_along_segment_m),
                "distance_along_route_m": float(distance_along_route_m),
                "snapped_x": float(snapped_point.x),
                "snapped_y": float(snapped_point.y)
            }

    return best_match


def get_curve_text(curvature_state):
    if curvature_state == "High Curvature":
        return "SHARP CURVE"
    elif curvature_state == "Moderate Curve":
        return "MODERATE CURVE"
    else:
        return "STRAIGHT"


def get_control_text(control_on_segment, intersection_warning):
    if control_on_segment == "Traffic signal on current segment":
        return "SIGNAL"
    elif control_on_segment == "Stop-controlled intersection on current segment":
        return "STOP"

    if intersection_warning is not None:
        if "Signalized" in intersection_warning:
            return "SIGNAL AHEAD"
        elif "Stop" in intersection_warning:
            return "STOP AHEAD"
        else:
            return "INTERSECTION"

    return "CLEAR"


def get_warning_level(curvature_state, control_on_segment, intersection_warning):
    if "Stop-controlled" in str(control_on_segment) or "Traffic signal" in str(control_on_segment):
        return "HIGH"

    if intersection_warning is not None:
        return "CAUTION"

    if curvature_state == "High Curvature":
        return "CAUTION"

    return "SAFE"


def get_warning_colors(level):
    if level == "HIGH":
        return {"bg": "#b91c1c", "fg": "white"}
    elif level == "CAUTION":
        return {"bg": "#f59e0b", "fg": "black"}
    else:
        return {"bg": "#15803d", "fg": "white"}


def get_warning_reason(control_on_segment, intersection_warning, curvature_state):
    if control_on_segment != "None":
        return control_on_segment
    if intersection_warning is not None:
        return intersection_warning
    if curvature_state == "High Curvature":
        return "Sharp curve ahead"
    if curvature_state == "Moderate Curve":
        return "Moderate curve"
    return "No immediate hazard"


def build_compact_display(
    speed_display_text,
    lane_display_text,
    road_type_class,
    advisory_speed_text,
    remaining_distance_miles,
    remaining_time_min
):
    parts = []

    parts.append(f"SPD {speed_display_text}")
    parts.append(f"LN {lane_display_text}")

    if road_type_class == "Highway":
        parts.append("HWY")
    elif road_type_class == "Rural/Arterial":
        parts.append("RURAL")
    else:
        parts.append("RES")

    parts.append(f"ADV {advisory_speed_text}")
    parts.append(f"{remaining_distance_miles:.2f} mi")

    if remaining_time_min is not None:
        parts.append(f"{remaining_time_min:.0f} min")

    return "   |   ".join(parts)


def classify_curvature(curvature_value):
    if curvature_value < 10:
        return "Straight"
    elif curvature_value < 30:
        return "Moderate Curve"
    else:
        return "High Curvature"


def classify_road_type(highway_value):
    if pd.isna(highway_value):
        return "Unknown"

    road = str(highway_value).lower()

    if road in ["motorway", "trunk", "primary"]:
        return "Highway"
    elif road in ["secondary", "tertiary"]:
        return "Rural/Arterial"
    elif road in ["residential", "service", "living_street"]:
        return "Residential"
    else:
        return road.title()


def classify_road_direction(direction_value):
    if str(direction_value).lower() == "oneway":
        return "One-way"
    else:
        return "Two-way"


def format_speed_display(segment):
    speed_kph = segment.get("maxspeed_kph")
    source = str(segment.get("speed_inference_source", ""))

    if speed_kph is None:
        return "--"

    speed_mph = kph_to_mph(speed_kph)

    if source == "speed_osm":
        return f"{speed_mph:.1f} mph"

    return f"{speed_mph:.1f} mph (inf)"


def format_lane_display(segment):
    lanes = segment.get("lanes_count")
    source = str(segment.get("lane_inference_source", ""))

    if lanes is None:
        return "Unknown"

    if source != "lanes_osm":
        return f"{lanes} (inferred)"

    return str(lanes)


def compute_speed_advisory(segment, curvature_state):
    speed = segment.get("maxspeed_kph")
    source = str(segment.get("speed_inference_source", ""))

    if source != "speed_osm" or speed is None:
        return None

    if curvature_state == "High Curvature":
        advisory = speed * 0.5
    elif curvature_state == "Moderate Curve":
        advisory = speed * 0.7
    else:
        advisory = speed

    return round(advisory / 5.0) * 5.0


def format_advisory_speed(speed_advisory_kph):
    if speed_advisory_kph is None:
        return "--"
    advisory_mph = kph_to_mph(speed_advisory_kph)
    return f"{advisory_mph:.0f} mph"


def classify_density(density_value):
    if density_value is None:
        return "Unknown"
    if density_value < 5:
        return "Low"
    elif density_value <= 15:
        return "Medium"
    else:
        return "High"


def get_upcoming_intersection_warning(current_distance_m, intersection_details, lookahead_m=150.0):
    upcoming = []

    for intersection in intersection_details:
        intersection_distance = intersection.get("distance_from_start_m")
        if intersection_distance is None:
            continue

        distance_ahead = intersection_distance - current_distance_m

        if 0 < distance_ahead <= lookahead_m:
            upcoming.append((distance_ahead, intersection))

    if not upcoming:
        return None

    upcoming.sort(key=lambda x: x[0])
    distance_ahead, nearest = upcoming[0]
    distance_ahead_miles = meters_to_miles(distance_ahead)

    if nearest.get("is_signalized"):
        return f"Signalized intersection ahead in {distance_ahead_miles:.2f} mi"
    elif nearest.get("is_stop_controlled"):
        return f"Stop-controlled intersection ahead in {distance_ahead_miles:.2f} mi"
    else:
        return f"Intersection ahead in {distance_ahead_miles:.2f} mi"


def get_control_on_current_segment(segment_start_m, segment_end_m, intersection_details, window_m=40.0):
    for intersection in intersection_details:
        intersection_distance = intersection.get("distance_from_start_m")
        if intersection_distance is None:
            continue

        if (segment_start_m - window_m) <= intersection_distance <= (segment_end_m + window_m):
            if intersection.get("is_signalized"):
                return "Traffic signal on current segment"
            elif intersection.get("is_stop_controlled"):
                return "Stop-controlled intersection on current segment"

    return "None"


def compute_remaining_distance_miles(total_length_miles, current_distance_m):
    current_distance_miles = meters_to_miles(current_distance_m)
    return max(total_length_miles - current_distance_miles, 0.0)


def compute_remaining_time_min(remaining_distance_miles, route_average_speed_mph):
    if route_average_speed_mph is None or route_average_speed_mph <= 0:
        return None
    return (remaining_distance_miles / route_average_speed_mph) * 60.0


def build_driver_alert_message(
    intersection_warning,
    control_on_segment,
    curvature_state,
    speed_display_text,
    lane_display_text,
    road_direction,
    road_type_class,
    advisory_speed_text,
    density_class,
    remaining_distance_miles,
    remaining_time_min
):
    symbol_line = get_symbols(
        curvature_state,
        intersection_warning,
        control_on_segment,
        road_direction,
        density_class
    )

    compact_line = build_compact_display(
        speed_display_text,
        lane_display_text,
        road_type_class,
        advisory_speed_text,
        remaining_distance_miles,
        remaining_time_min
    )

    return f"{symbol_line}\n{compact_line}"


def get_segment_by_id(segment_id, edge_attributes):
    for segment in edge_attributes:
        if int(segment.get("segment_id")) == int(segment_id):
            return segment
    return None


with open("output/US_190_to_FM_2038_to_FM_244.json", "r") as f:
    route_data = json.load(f)

odom_df = pd.read_csv("rosbag_perception_output_planning_2025-09-30_11-36-38_2025-09-30-11-41-54_1_novatel_odom_data.csv")

edge_attributes = route_data["edge_attributes"]
intersection_details = route_data["intersection_details"]
route_metrics = route_data["route_metrics"]
waypoint_density_summary = route_data["waypoint_density_summary"]
edge_attribute_summary = route_data["edge_attribute_summary"]
points_of_interest = route_data["points_of_interest"]

root = tk.Tk()
root.title("Driver Alert System")
root.geometry("1480x840")
root.minsize(1280, 760)
root.configure(bg="white")

title_label = tk.Label(
    root,
    text="Driver Assistance System",
    font=("DejaVu Sans", 24, "bold"),
    bg="#1f3b5c",
    fg="white",
    pady=10
)
title_label.pack(fill="x", pady=(0, 8))

warning_frame = tk.Frame(root, bg="#202020", bd=0, relief="flat")
warning_frame.pack(fill="x", padx=24, pady=(8, 8))

warning_label = tk.Label(
    warning_frame,
    text="SAFE",
    font=("DejaVu Sans", 28, "bold"),
    fg="white",
    bg="#15803d",
    pady=14
)
warning_label.pack(fill="x")

symbol_label = tk.Label(
    root,
    text="",
    font=("DejaVu Sans", 18, "bold"),
    bg="white",
    fg="#1f2937",
    pady=6
)
symbol_label.pack(fill="x", padx=24, pady=(0, 8))

status_frame = tk.Frame(root, bg="white")
status_frame.pack(fill="x", padx=24, pady=10)

status_frame.grid_columnconfigure(0, weight=1)
status_frame.grid_columnconfigure(1, weight=1)
status_frame.grid_columnconfigure(2, weight=1)
status_frame.grid_columnconfigure(3, weight=2)
status_frame.grid_columnconfigure(4, weight=1)


def make_status_card(parent, title, wraplength=260, value_font=("DejaVu Sans", 18, "bold")):
    card = tk.Frame(parent, bg="white", bd=1, relief="solid")

    header = tk.Label(
        card,
        text=title,
        font=("DejaVu Sans", 13, "bold"),
        bg="white",
        fg="#4b5563"
    )
    header.pack(pady=(10, 4))

    value = tk.Label(
        card,
        text="--",
        font=value_font,
        bg="white",
        fg="#111827",
        wraplength=wraplength,
        justify="center"
    )
    value.pack(pady=(0, 12), padx=10)

    return card, value


speed_card, speed_box = make_status_card(status_frame, "SPEED LIMIT", wraplength=180, value_font=("DejaVu Sans", 20, "bold"))
speed_card.grid(row=0, column=0, sticky="nsew", padx=8, pady=6)

advisory_card, advisory_box = make_status_card(status_frame, "ADVISORY", wraplength=180, value_font=("DejaVu Sans", 20, "bold"))
advisory_card.grid(row=0, column=1, sticky="nsew", padx=8, pady=6)

curve_card, curve_box = make_status_card(status_frame, "CURVE", wraplength=220, value_font=("DejaVu Sans", 20, "bold"))
curve_card.grid(row=0, column=2, sticky="nsew", padx=8, pady=6)

control_card, control_box = make_status_card(status_frame, "CONTROL", wraplength=360, value_font=("DejaVu Sans", 17, "bold"))
control_card.grid(row=0, column=3, sticky="nsew", padx=8, pady=6)

distance_card, distance_box = make_status_card(status_frame, "REMAIN ROUTE", wraplength=220, value_font=("DejaVu Sans", 20, "bold"))
distance_card.grid(row=0, column=4, sticky="nsew", padx=8, pady=6)

info_frame = tk.Frame(root, bg="white", bd=1, relief="solid")
info_frame.pack(fill="both", expand=True, padx=24, pady=(12, 20))

details_title = tk.Label(
    info_frame,
    text="Detailed Route / Segment Information",
    font=("DejaVu Sans", 16, "bold"),
    bg="white",
    fg="#374151",
    anchor="w"
)
details_title.pack(fill="x", padx=14, pady=(12, 8))

segment_info_label = tk.Label(
    info_frame,
    text="",
    font=("DejaVu Sans Mono", 14),
    justify="left",
    anchor="nw",
    bg="white",
    fg="#111827"
)
segment_info_label.pack(fill="both", expand=True, padx=14, pady=(0, 14))

driver_state_rows = []


def update_driver_alert_gui(output_row, driver_alert_message):
    warning_level = get_warning_level(
        output_row["curvature_state"],
        output_row["control_on_segment"],
        output_row["intersection_warning"]
    )

    colors = get_warning_colors(warning_level)

    warning_reason = get_warning_reason(
        output_row["control_on_segment"],
        output_row["intersection_warning"],
        output_row["curvature_state"]
    )

    if warning_level == "HIGH":
        warning_text = f"HIGH ALERT  |  {warning_reason}"
    elif warning_level == "CAUTION":
        warning_text = f"CAUTION  |  {warning_reason}"
    else:
        warning_text = f"SAFE  |  {warning_reason}"

    warning_label.config(
        text=warning_text,
        bg=colors["bg"],
        fg=colors["fg"]
    )
    warning_frame.config(bg=colors["bg"])
    symbol_label.config(text=driver_alert_message)

    speed_box.config(
        text=output_row["speed_display_text"],
        bg="white",
        fg="#111827" if output_row["speed_display_text"] != "--" else "#6b7280"
    )

    advisory_box.config(
        text=output_row["advisory_speed_text"],
        bg="white",
        fg="#111827" if output_row["advisory_speed_text"] != "--" else "#6b7280"
    )

    curve_box.config(
        text=get_curve_text(output_row["curvature_state"]),
        bg="white",
        fg="#111827"
    )

    control_box.config(
        text=get_control_text(output_row["control_on_segment"], output_row["intersection_warning"]),
        bg="white",
        fg="#111827"
    )

    distance_box.config(
        text=f"{output_row['remaining_distance_miles']:.2f} mi",
        bg="white",
        fg="#111827"
    )

    segment_text = (
        f"Segment ID              : {output_row['segment_id']}\n"
        f"Road Type               : {output_row['road_type_class']}\n"
        f"Road Direction          : {output_row['road_direction']}\n"
        f"Lane Count              : {output_row['lane_display_text']}\n"
        f"Curvature State         : {output_row['curvature_state']}\n"
        f"Curvature Value         : {output_row['segment_curvature_value']:.2f}\n"
        f"Speed Limit             : {output_row['speed_display_text']}\n"
        f"Advisory Speed          : {output_row['advisory_speed_text']}\n"
        f"Current Control         : {output_row['control_on_segment']}\n"
        f"Upcoming Intersection   : {output_row['intersection_warning']}\n"
        f"Waypoint Density        : {output_row['density_class']}\n"
        f"Distance to Segment     : {output_row['distance_to_segment_miles']:.3f} mi\n"
        f"Distance Along Route    : {output_row['distance_along_route_miles']:.3f} mi\n"
        f"Remaining Full Route    : {output_row['remaining_distance_miles']:.3f} mi\n"
        f"Remaining Time          : {output_row['remaining_time_min']:.1f} min"
    )

    segment_info_label.config(text=segment_text)
    root.update()


route_average_speed_kph = edge_attribute_summary.get("weighted_avg_speed_limit_kph")
route_average_speed_mph = None if route_average_speed_kph is None else kph_to_mph(route_average_speed_kph)

total_length_km = route_metrics["length_km"]
total_length_miles = km_to_miles(total_length_km)

segments_gdf, projected_crs = prepare_segment_geometries(edge_attributes)

last_distance_m = None

for _, odom_row in odom_df.iterrows():
    vehicle_x = odom_row["position_x"]
    vehicle_y = odom_row["position_y"]

    match_info = locate_vehicle_on_route_segment(vehicle_x, vehicle_y, segments_gdf)
    if match_info is None:
        continue

    current_distance_m = match_info["distance_along_route_m"]
    segment_id = match_info["segment_id"]

    if last_distance_m is not None and abs(current_distance_m - last_distance_m) < 2.0:
        continue

    segment = get_segment_by_id(segment_id, edge_attributes)
    if segment is None:
        continue

    segment_start_m = segment.get("segment_start_m", 0.0)
    segment_end_m = segment.get("segment_end_m", 0.0)

    curvature_value = segment.get("curvature_deg") or 0.0
    curvature_state = classify_curvature(curvature_value)

    road_type = segment.get("primary_highway")
    road_type_class = classify_road_type(road_type)

    road_direction = classify_road_direction(segment.get("direction"))

    speed_display_text = format_speed_display(segment)
    lane_display_text = format_lane_display(segment)

    speed_advisory_kph = compute_speed_advisory(segment, curvature_state)
    advisory_speed_text = format_advisory_speed(speed_advisory_kph)

    density_value = segment.get("waypoint_density_per_km")
    density_class = classify_density(density_value)

    intersection_warning = get_upcoming_intersection_warning(
        current_distance_m,
        intersection_details,
        lookahead_m=150.0
    )

    control_on_segment = get_control_on_current_segment(
        segment_start_m,
        segment_end_m,
        intersection_details,
        window_m=40.0
    )

    remaining_distance_miles = compute_remaining_distance_miles(total_length_miles, current_distance_m)
    remaining_time_min = compute_remaining_time_min(remaining_distance_miles, route_average_speed_mph)

    output_row = {
        "segment_id": segment.get("segment_id"),
        "road_type": road_type,
        "road_type_class": road_type_class,
        "current_speed_kph": segment.get("maxspeed_kph"),
        "speed_display_text": speed_display_text,
        "lane_count": segment.get("lanes_count"),
        "lane_display_text": lane_display_text,
        "road_direction": road_direction,
        "segment_curvature_value": curvature_value,
        "curvature_state": curvature_state,
        "control_on_segment": control_on_segment,
        "intersection_warning": intersection_warning,
        "speed_advisory_kph": speed_advisory_kph,
        "advisory_speed_text": advisory_speed_text,
        "waypoint_density_per_km": density_value,
        "density_class": density_class,
        "remaining_distance_miles": remaining_distance_miles,
        "remaining_time_min": remaining_time_min,
        "distance_to_segment_miles": meters_to_miles(match_info["distance_to_segment_m"]),
        "distance_along_route_miles": meters_to_miles(match_info["distance_along_route_m"])
    }

    driver_alert_message = build_driver_alert_message(
        intersection_warning,
        control_on_segment,
        curvature_state,
        speed_display_text,
        lane_display_text,
        road_direction,
        road_type_class,
        advisory_speed_text,
        density_class,
        remaining_distance_miles,
        remaining_time_min
    )

    update_driver_alert_gui(output_row, driver_alert_message)

    driver_state_rows.append(output_row)
    last_distance_m = current_distance_m

    root.update_idletasks()
    root.update()
    time.sleep(0.0)

driver_state_df = pd.DataFrame(driver_state_rows)
driver_state_df.to_csv("output/driver_state_table_segment_only.csv", index=False)

print(driver_state_df.head(20))

root.mainloop()
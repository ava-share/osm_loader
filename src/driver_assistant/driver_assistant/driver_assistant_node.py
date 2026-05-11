#!/usr/bin/env python3

import pandas as pd
import tkinter as tk
import json
from shapely import wkt
from shapely.geometry import Point
import geopandas as gpd
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


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


def project_intersections(intersection_details, target_crs):
    rows = []
    for i, item in enumerate(intersection_details):
        lon = item.get("longitude")
        lat = item.get("latitude")
        if lon is None or lat is None:
            continue
        rows.append({
            "idx": i,
            "is_signalized": item.get("is_signalized", False),
            "is_stop_controlled": item.get("is_stop_controlled", False),
            "geometry": Point(lon, lat),
        })

    gdf = gpd.GeoDataFrame(rows, geometry="geometry", crs="EPSG:4326")
    return gdf.to_crs(target_crs)


class RosBridge(Node):
    def __init__(self, segments_gdf, intersections_gdf, origin_x, origin_y):
        super().__init__("driver_assistant_ros_bridge")

        self.origin_x = origin_x
        self.origin_y = origin_y
        self.segments_gdf = segments_gdf
        self.intersections_gdf = intersections_gdf

        self.path_pub = self.create_publisher(Path, "/osm_route_path", 10)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odom", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/osm_route_markers", 10)
        self.alert_pub = self.create_publisher(String, "/driver_alert_text", 10)

        self.route_path_msg = self._build_path_msg()
        self.marker_array_msg = self._build_marker_array()

        self.create_timer(1.0, self._publish_static_layers)

    def _build_path_msg(self):
        msg = Path()
        msg.header.frame_id = "map"

        for _, row in self.segments_gdf.iterrows():
            line = row.geometry
            for x, y in line.coords:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x) - self.origin_x
                pose.pose.position.y = float(y) - self.origin_y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)

        return msg

    def _build_marker_array(self):
        marker_array = MarkerArray()

        for _, row in self.intersections_gdf.iterrows():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "route_intersections"
            marker.id = int(row["idx"])
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(row.geometry.x) - self.origin_x
            marker.pose.position.y = float(row.geometry.y) - self.origin_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 12.0
            marker.scale.y = 12.0
            marker.scale.z = 12.0

            if row["is_signalized"]:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
            elif row["is_stop_controlled"]:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
            marker.color.a = 1.0

            marker_array.markers.append(marker)

        return marker_array

    def _publish_static_layers(self):
        now = self.get_clock().now().to_msg()

        self.route_path_msg.header.stamp = now
        for pose in self.route_path_msg.poses:
            pose.header.stamp = now
        for marker in self.marker_array_msg.markers:
            marker.header.stamp = now

        self.path_pub.publish(self.route_path_msg)
        self.marker_pub.publish(self.marker_array_msg)

    def publish_vehicle(self, position_x, position_y, alert_text):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(position_x) - self.origin_x
        msg.pose.pose.position.y = float(position_y) - self.origin_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

        text = String()
        text.data = alert_text
        self.alert_pub.publish(text)


def lane_bars(lanes_count):
    if lanes_count is None:
        return "?"
    try:
        n = int(lanes_count)
    except (TypeError, ValueError):
        return "?"
    if n <= 0:
        return "?"
    if n > 8:
        n = 8
    return "│" * n


def curve_glyph(curvature_state):
    if curvature_state == "High Curvature":
        return "↷"
    if curvature_state == "Moderate Curve":
        return "↱"
    return "→"


def control_glyph_and_color(control_on_segment, intersection_warning):
    if control_on_segment == "Traffic signal on current segment":
        return "●", "#dc2626"
    if control_on_segment == "Stop-controlled intersection on current segment":
        return "⬢", "#b91c1c"
    if intersection_warning is not None:
        if "Signalized" in intersection_warning:
            return "◐", "#f59e0b"
        if "Stop" in intersection_warning:
            return "⬣", "#f59e0b"
        return "✚", "#f59e0b"
    return "✓", "#15803d"


def direction_glyph(road_direction):
    if road_direction == "One-way":
        return "➡"
    return "⇄"


def density_glyph(density_class):
    if density_class == "High":
        return "≋"
    if density_class == "Medium":
        return "≈"
    if density_class == "Low":
        return "—"
    return "?"


def warning_level_glyph(warning_level):
    if warning_level == "HIGH":
        return "⚠"
    if warning_level == "CAUTION":
        return "⚠"
    return "✓"


def verify_route_position(match_info, segment, current_distance_m, last_distance_m, last_segment_id):
    issues = []
    snap_ok = True
    progress_ok = True
    continuity_ok = True

    snap_dist = float(match_info["distance_to_segment_m"])
    if snap_dist > 30.0:
        snap_ok = False
        issues.append(f"OFF ROUTE ({snap_dist:.1f} m)")
    elif snap_dist > 10.0:
        snap_ok = False
        issues.append(f"WEAK SNAP ({snap_dist:.1f} m)")

    delta_m = None
    if last_distance_m is not None:
        delta_m = current_distance_m - last_distance_m
        if delta_m < -5.0:
            progress_ok = False
            issues.append(f"REVERSE ({delta_m:.1f} m)")

    seg_start = float(segment.get("segment_start_m", 0.0))
    seg_end = float(segment.get("segment_end_m", 0.0))
    if not (seg_start - 5.0 <= current_distance_m <= seg_end + 5.0):
        continuity_ok = False
        issues.append("OUT OF SEGMENT RANGE")

    sid_now = segment.get("segment_id")
    if last_segment_id is not None and sid_now is not None:
        try:
            if int(sid_now) < int(last_segment_id) - 1:
                continuity_ok = False
                issues.append(f"SEGMENT REGRESSION {last_segment_id}->{sid_now}")
        except (TypeError, ValueError):
            pass

    return {
        "snap_distance_m": snap_dist,
        "delta_m": delta_m,
        "snap_ok": snap_ok,
        "progress_ok": progress_ok,
        "continuity_ok": continuity_ok,
        "issues": issues,
        "overall_ok": snap_ok and progress_ok and continuity_ok,
    }


def main(args=None):
    rclpy.init(args=args)

    # Parameter node to read file paths before building the bridge
    class _Params(Node):
        def __init__(self):
            super().__init__("driver_assistant_params")
            self.declare_parameter(
                "route_file",
                "/home/avalocal/Downloads/osm_loader/output/FM_1362_to_FM_2000.json",
            )
            self.declare_parameter(
                "odom_csv",
                "/home/avalocal/Downloads/osm_loader/combined_novatel_odom_data.csv",
            )

    params = _Params()
    route_file = params.get_parameter("route_file").get_parameter_value().string_value
    odom_csv = params.get_parameter("odom_csv").get_parameter_value().string_value
    params.destroy_node()

    with open(route_file, "r") as f:
        route_data = json.load(f)

    odom_df = pd.read_csv(odom_csv)

    edge_attributes = route_data["edge_attributes"]
    intersection_details = route_data["intersection_details"]
    route_metrics = route_data["route_metrics"]
    waypoint_density_summary = route_data["waypoint_density_summary"]
    edge_attribute_summary = route_data["edge_attribute_summary"]
    points_of_interest = route_data["points_of_interest"]

    root = tk.Tk()
    root.title("Driver Assistance System")
    root.geometry("1480x900")
    root.minsize(1280, 800)
    root.configure(bg="#f3f4f6")

    title_label = tk.Label(
        root,
        text="DRIVER ASSISTANCE",
        font=("DejaVu Sans", 22, "bold"),
        bg="#1f3b5c",
        fg="white",
        pady=10,
    )
    title_label.pack(fill="x")

    warning_frame = tk.Frame(root, bg="#15803d")
    warning_frame.pack(fill="x", padx=16, pady=(10, 6))

    warning_glyph_label = tk.Label(
        warning_frame,
        text="✓",
        font=("DejaVu Sans", 48, "bold"),
        bg="#15803d",
        fg="white",
        width=3,
    )
    warning_glyph_label.pack(side="left", padx=(20, 12), pady=10)

    warning_label = tk.Label(
        warning_frame,
        text="SAFE",
        font=("DejaVu Sans", 26, "bold"),
        fg="white",
        bg="#15803d",
        anchor="w",
        justify="left",
    )
    warning_label.pack(side="left", fill="both", expand=True, pady=10)

    symbol_row = tk.Frame(root, bg="#f3f4f6")
    symbol_row.pack(fill="x", padx=16, pady=(4, 8))
    for i in range(6):
        symbol_row.grid_columnconfigure(i, weight=1, uniform="cards")

    def make_symbol_card(parent, header, glyph_font_size=44):
        card = tk.Frame(parent, bg="white", bd=1, relief="solid")
        head = tk.Label(
            card,
            text=header,
            font=("DejaVu Sans", 11, "bold"),
            bg="white",
            fg="#6b7280",
        )
        head.pack(pady=(8, 2))
        glyph = tk.Label(
            card,
            text="—",
            font=("DejaVu Sans", glyph_font_size, "bold"),
            bg="white",
            fg="#111827",
        )
        glyph.pack()
        sub = tk.Label(
            card,
            text="--",
            font=("DejaVu Sans", 13, "bold"),
            bg="white",
            fg="#374151",
        )
        sub.pack(pady=(0, 8))
        return card, glyph, sub

    curve_card, curve_glyph_label, curve_sub_label = make_symbol_card(symbol_row, "CURVE")
    curve_card.grid(row=0, column=0, sticky="nsew", padx=4, pady=4)

    control_card, control_glyph_label, control_sub_label = make_symbol_card(symbol_row, "CONTROL")
    control_card.grid(row=0, column=1, sticky="nsew", padx=4, pady=4)

    direction_card, direction_glyph_label, direction_sub_label = make_symbol_card(symbol_row, "DIRECTION")
    direction_card.grid(row=0, column=2, sticky="nsew", padx=4, pady=4)

    lane_card, lane_glyph_label, lane_sub_label = make_symbol_card(symbol_row, "LANES", glyph_font_size=40)
    lane_card.grid(row=0, column=3, sticky="nsew", padx=4, pady=4)

    density_card, density_glyph_label, density_sub_label = make_symbol_card(symbol_row, "DENSITY")
    density_card.grid(row=0, column=4, sticky="nsew", padx=4, pady=4)

    speed_card = tk.Frame(symbol_row, bg="white", bd=1, relief="solid")
    speed_card.grid(row=0, column=5, sticky="nsew", padx=4, pady=4)
    speed_header = tk.Label(
        speed_card,
        text="SPEED  /  ADVISORY",
        font=("DejaVu Sans", 11, "bold"),
        bg="white",
        fg="#6b7280",
    )
    speed_header.pack(pady=(8, 2))
    speed_pair = tk.Frame(speed_card, bg="white")
    speed_pair.pack(fill="x", padx=6, pady=(0, 8))
    speed_value_label = tk.Label(
        speed_pair, text="--", font=("DejaVu Sans", 22, "bold"), bg="white", fg="#111827"
    )
    speed_value_label.pack(side="left", expand=True)
    speed_divider = tk.Label(
        speed_pair, text="│", font=("DejaVu Sans", 22, "bold"), bg="white", fg="#9ca3af"
    )
    speed_divider.pack(side="left", padx=4)
    advisory_value_label = tk.Label(
        speed_pair, text="--", font=("DejaVu Sans", 22, "bold"), bg="white", fg="#111827"
    )
    advisory_value_label.pack(side="left", expand=True)

    verify_frame = tk.Frame(root, bg="#f3f4f6")
    verify_frame.pack(fill="x", padx=16, pady=(2, 6))

    def make_badge(parent, label):
        f = tk.Frame(parent, bg="#15803d", bd=0)
        text = tk.Label(
            f,
            text=f"{label}: ✓",
            font=("DejaVu Sans Mono", 12, "bold"),
            bg="#15803d",
            fg="white",
            padx=10,
            pady=4,
        )
        text.pack()
        return f, text

    snap_badge, snap_badge_text = make_badge(verify_frame, "SNAP")
    snap_badge.pack(side="left", padx=(0, 6))
    progress_badge, progress_badge_text = make_badge(verify_frame, "PROGRESS")
    progress_badge.pack(side="left", padx=6)
    continuity_badge, continuity_badge_text = make_badge(verify_frame, "CONTINUITY")
    continuity_badge.pack(side="left", padx=6)
    segment_badge, segment_badge_text = make_badge(verify_frame, "SEG")
    segment_badge.pack(side="left", padx=6)
    distance_badge, distance_badge_text = make_badge(verify_frame, "REMAIN")
    distance_badge.pack(side="right", padx=(6, 0))

    info_frame = tk.Frame(root, bg="white", bd=1, relief="solid")
    info_frame.pack(fill="both", expand=True, padx=16, pady=(6, 14))

    details_title = tk.Label(
        info_frame,
        text="Segment Detail",
        font=("DejaVu Sans", 14, "bold"),
        bg="white",
        fg="#374151",
        anchor="w",
    )
    details_title.pack(fill="x", padx=14, pady=(10, 6))

    segment_info_label = tk.Label(
        info_frame,
        text="",
        font=("DejaVu Sans Mono", 13),
        justify="left",
        anchor="nw",
        bg="white",
        fg="#111827",
    )
    segment_info_label.pack(fill="both", expand=True, padx=14, pady=(0, 12))

    driver_state_rows = []

    def _style_badge(badge_frame, badge_text_label, ok, label, value):
        color = "#15803d" if ok else "#b91c1c"
        badge_frame.config(bg=color)
        badge_text_label.config(bg=color, text=f"{label}: {value}")

    def update_driver_alert_gui(output_row, verification_info):
        warning_level = get_warning_level(
            output_row["curvature_state"],
            output_row["control_on_segment"],
            output_row["intersection_warning"],
        )
        colors = get_warning_colors(warning_level)
        warning_reason = get_warning_reason(
            output_row["control_on_segment"],
            output_row["intersection_warning"],
            output_row["curvature_state"],
        )

        if warning_level == "HIGH":
            warning_text = f"HIGH ALERT  |  {warning_reason}"
        elif warning_level == "CAUTION":
            warning_text = f"CAUTION  |  {warning_reason}"
        else:
            warning_text = f"SAFE  |  {warning_reason}"

        warning_frame.config(bg=colors["bg"])
        warning_glyph_label.config(text=warning_level_glyph(warning_level), bg=colors["bg"], fg=colors["fg"])
        warning_label.config(text=warning_text, bg=colors["bg"], fg=colors["fg"])

        curve_glyph_label.config(text=curve_glyph(output_row["curvature_state"]))
        curve_sub_label.config(text=f"{output_row['segment_curvature_value']:.1f}°")

        glyph_text, glyph_color = control_glyph_and_color(
            output_row["control_on_segment"], output_row["intersection_warning"]
        )
        control_glyph_label.config(text=glyph_text, fg=glyph_color)
        control_sub_label.config(
            text=get_control_text(
                output_row["control_on_segment"], output_row["intersection_warning"]
            )
        )

        direction_glyph_label.config(text=direction_glyph(output_row["road_direction"]))
        direction_sub_label.config(text=output_row["road_direction"])

        lane_glyph_label.config(text=lane_bars(output_row["lane_count"]))
        lane_sub_label.config(text=output_row["lane_display_text"])

        density_glyph_label.config(text=density_glyph(output_row["density_class"]))
        density_sub_label.config(text=output_row["density_class"])

        speed_value_label.config(
            text=output_row["speed_display_text"],
            fg="#111827" if output_row["speed_display_text"] != "--" else "#6b7280",
        )
        advisory_value_label.config(
            text=output_row["advisory_speed_text"],
            fg="#111827" if output_row["advisory_speed_text"] != "--" else "#6b7280",
        )

        _style_badge(
            snap_badge, snap_badge_text,
            verification_info["snap_ok"],
            "SNAP",
            f"{verification_info['snap_distance_m']:.1f} m",
        )
        _style_badge(
            progress_badge, progress_badge_text,
            verification_info["progress_ok"],
            "PROGRESS",
            "OK" if verification_info["progress_ok"] else "REVERSE",
        )
        _style_badge(
            continuity_badge, continuity_badge_text,
            verification_info["continuity_ok"],
            "CONTINUITY",
            "OK" if verification_info["continuity_ok"] else "JUMP",
        )
        _style_badge(
            segment_badge, segment_badge_text,
            True,
            "SEG",
            str(output_row["segment_id"]),
        )
        _style_badge(
            distance_badge, distance_badge_text,
            True,
            "REMAIN",
            f"{output_row['remaining_distance_miles']:.2f} mi",
        )

        issues_str = " | ".join(verification_info["issues"]) if verification_info["issues"] else "OK"

        segment_text = (
            f"Segment ID              : {output_row['segment_id']}\n"
            f"Road Type               : {output_row['road_type_class']}\n"
            f"Road Direction          : {output_row['road_direction']}\n"
            f"Lane Count              : {output_row['lane_display_text']}\n"
            f"Curvature                : {output_row['curvature_state']} ({output_row['segment_curvature_value']:.2f}°)\n"
            f"Speed Limit             : {output_row['speed_display_text']}\n"
            f"Advisory Speed          : {output_row['advisory_speed_text']}\n"
            f"Current Control         : {output_row['control_on_segment']}\n"
            f"Upcoming Intersection   : {output_row['intersection_warning']}\n"
            f"Waypoint Density        : {output_row['density_class']}\n"
            f"Distance to Segment     : {output_row['distance_to_segment_miles']:.3f} mi\n"
            f"Distance Along Route    : {output_row['distance_along_route_miles']:.3f} mi\n"
            f"Remaining               : {output_row['remaining_distance_miles']:.3f} mi  /  {output_row['remaining_time_min']:.1f} min\n"
            f"VERIFICATION            : {issues_str}"
        )

        segment_info_label.config(text=segment_text)
        root.update()

    route_average_speed_kph = edge_attribute_summary.get("weighted_avg_speed_limit_kph")
    route_average_speed_mph = None if route_average_speed_kph is None else kph_to_mph(route_average_speed_kph)

    total_length_km = route_metrics["length_km"]
    total_length_miles = km_to_miles(total_length_km)

    segments_gdf, projected_crs = prepare_segment_geometries(edge_attributes)

    intersections_gdf_for_ros = project_intersections(intersection_details, projected_crs)

    origin_x = float(odom_df.iloc[0]["position_x"])
    origin_y = float(odom_df.iloc[0]["position_y"])

    ros_bridge = RosBridge(segments_gdf, intersections_gdf_for_ros, origin_x, origin_y)

    last_distance_m = None
    last_segment_id = None

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

        verification_info = verify_route_position(
            match_info,
            segment,
            current_distance_m,
            last_distance_m,
            last_segment_id,
        )

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
            "distance_along_route_miles": meters_to_miles(match_info["distance_along_route_m"]),
            "verify_snap_ok": verification_info["snap_ok"],
            "verify_progress_ok": verification_info["progress_ok"],
            "verify_continuity_ok": verification_info["continuity_ok"],
            "verify_overall_ok": verification_info["overall_ok"],
            "verify_issues": "; ".join(verification_info["issues"]),
        }

        update_driver_alert_gui(output_row, verification_info)

        ros_warning_level = get_warning_level(curvature_state, control_on_segment, intersection_warning)
        ros_warning_reason = get_warning_reason(control_on_segment, intersection_warning, curvature_state)
        verify_tag = "OK" if verification_info["overall_ok"] else "VERIFY:" + ",".join(verification_info["issues"])
        ros_alert_text = f"Seg {segment.get('segment_id')} | {ros_warning_level} | {ros_warning_reason} | {verify_tag}"
        ros_bridge.publish_vehicle(vehicle_x, vehicle_y, ros_alert_text)
        rclpy.spin_once(ros_bridge, timeout_sec=0.0)

        driver_state_rows.append(output_row)
        last_distance_m = current_distance_m
        last_segment_id = segment.get("segment_id")

        root.update_idletasks()
        root.update()

    driver_state_df = pd.DataFrame(driver_state_rows)
    driver_state_df.to_csv("output/driver_state_table_segment_only.csv", index=False)

    print(driver_state_df.head(20))

    try:
        while rclpy.ok():
            rclpy.spin_once(ros_bridge, timeout_sec=0.0)
            root.update_idletasks()
            root.update()
    except (tk.TclError, KeyboardInterrupt):
        pass

    ros_bridge.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

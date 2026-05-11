#!/usr/bin/env python3

import json

import geopandas as gpd
import pandas as pd
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry, Path
from rclpy.node import Node
from shapely import wkt
from shapely.geometry import Point
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray


def build_projected_route_and_intersections(route_data):
    edge_attributes = route_data["edge_attributes"]
    intersection_details = route_data["intersection_details"]

    route_rows = [
        {"segment_id": s.get("segment_id"), "geometry": wkt.loads(s["segment_geometry_wkt"])}
        for s in edge_attributes
        if s.get("segment_geometry_wkt")
    ]
    route_gdf = gpd.GeoDataFrame(route_rows, geometry="geometry", crs="EPSG:4326")
    crs = route_gdf.estimate_utm_crs() or "EPSG:3857"
    route_gdf = route_gdf.to_crs(crs)

    int_rows = [
        {
            "idx": i,
            "is_signalized": item.get("is_signalized", False),
            "is_stop_controlled": item.get("is_stop_controlled", False),
            "distance_from_start_m": item.get("distance_from_start_m"),
            "geometry": Point(item["longitude"], item["latitude"]),
        }
        for i, item in enumerate(intersection_details)
        if item.get("longitude") is not None and item.get("latitude") is not None
    ]
    intersections_gdf = gpd.GeoDataFrame(int_rows, geometry="geometry", crs="EPSG:4326")
    intersections_gdf = intersections_gdf.to_crs(crs)

    return route_gdf, intersections_gdf, crs


class CsvMapvizPlayer(Node):
    def __init__(self):
        super().__init__("csv_mapviz_player")

        self.declare_parameter(
            "route_file",
            "/home/avalocal/Downloads/osm_loader/output/FM_1362_to_FM_2000.json",
        )
        self.declare_parameter(
            "odom_csv",
            "/home/avalocal/Downloads/osm_loader/combined_novatel_odom_data.csv",
        )
        self.declare_parameter("publish_rate_hz", 10.0)

        route_file = self.get_parameter("route_file").get_parameter_value().string_value
        odom_csv = self.get_parameter("odom_csv").get_parameter_value().string_value
        publish_rate = self.get_parameter("publish_rate_hz").get_parameter_value().double_value

        with open(route_file) as f:
            route_data = json.load(f)

        self.odom_df = pd.read_csv(odom_csv)
        self.route_gdf, self.intersections_gdf, _ = build_projected_route_and_intersections(route_data)

        self.origin_x = float(self.odom_df.iloc[0]["position_x"])
        self.origin_y = float(self.odom_df.iloc[0]["position_y"])

        self.path_pub = self.create_publisher(Path, "/osm_route_path", 10)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odom", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/osm_route_markers", 10)
        self.alert_pub = self.create_publisher(String, "/driver_alert_text", 10)

        self.route_path = self._build_path_msg()
        self.marker_array = self._build_marker_array()

        self.index = 0

        self.create_timer(1.0, self._publish_static_layers)
        self.create_timer(1.0 / publish_rate, self._publish_vehicle)

    def _build_path_msg(self):
        msg = Path()
        msg.header.frame_id = "map"
        for _, row in self.route_gdf.iterrows():
            for x, y in row.geometry.coords:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x) - self.origin_x
                pose.pose.position.y = float(y) - self.origin_y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)
        return msg

    def _build_marker_array(self):
        arr = MarkerArray()
        for _, row in self.intersections_gdf.iterrows():
            m = Marker()
            m.header.frame_id = "map"
            m.ns = "route_intersections"
            m.id = int(row["idx"])
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(row.geometry.x) - self.origin_x
            m.pose.position.y = float(row.geometry.y) - self.origin_y
            m.pose.position.z = 0.0
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 12.0
            if row["is_signalized"]:
                m.color.r, m.color.g, m.color.b = 1.0, 1.0, 0.0
            elif row["is_stop_controlled"]:
                m.color.r, m.color.g, m.color.b = 1.0, 0.0, 0.0
            else:
                m.color.r, m.color.g, m.color.b = 0.0, 0.5, 1.0
            m.color.a = 1.0
            arr.markers.append(m)
        return arr

    def _publish_static_layers(self):
        now = self.get_clock().now().to_msg()
        self.route_path.header.stamp = now
        for pose in self.route_path.poses:
            pose.header.stamp = now
        for marker in self.marker_array.markers:
            marker.header.stamp = now
        self.path_pub.publish(self.route_path)
        self.marker_pub.publish(self.marker_array)

    def _publish_vehicle(self):
        if self.index >= len(self.odom_df):
            self.get_logger().info("CSV playback complete.", once=True)
            return

        row = self.odom_df.iloc[self.index]

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(row["position_x"]) - self.origin_x
        msg.pose.pose.position.y = float(row["position_y"]) - self.origin_y
        msg.pose.pose.position.z = 0.0
        msg.pose.pose.orientation.w = 1.0
        self.odom_pub.publish(msg)

        text = String()
        text.data = f"CSV index: {self.index}"
        self.alert_pub.publish(text)

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = CsvMapvizPlayer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()

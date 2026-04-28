#!/usr/bin/env python3

import json
import pandas as pd
import geopandas as gpd
from shapely import wkt
from shapely.geometry import Point

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path, Odometry
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import String


def build_projected_route_and_intersections(route_data):
    edge_attributes = route_data["edge_attributes"]
    intersection_details = route_data["intersection_details"]

    route_rows = []
    for seg in edge_attributes:
        geom_wkt = seg.get("segment_geometry_wkt")
        if geom_wkt is None:
            continue
        route_rows.append({
            "segment_id": seg.get("segment_id"),
            "geometry": wkt.loads(geom_wkt)
        })

    route_gdf = gpd.GeoDataFrame(route_rows, geometry="geometry", crs="EPSG:4326")
    projected_crs = route_gdf.estimate_utm_crs()
    if projected_crs is None:
        projected_crs = "EPSG:3857"
    route_gdf = route_gdf.to_crs(projected_crs)

    int_rows = []
    for i, item in enumerate(intersection_details):
        lon = item.get("longitude")
        lat = item.get("latitude")
        if lon is None or lat is None:
            continue
        int_rows.append({
            "idx": i,
            "is_signalized": item.get("is_signalized", False),
            "is_stop_controlled": item.get("is_stop_controlled", False),
            "distance_from_start_m": item.get("distance_from_start_m"),
            "geometry": Point(lon, lat)
        })

    intersections_gdf = gpd.GeoDataFrame(int_rows, geometry="geometry", crs="EPSG:4326")
    intersections_gdf = intersections_gdf.to_crs(projected_crs)

    return route_gdf, intersections_gdf, projected_crs


class CsvMapvizPlayer(Node):
    def __init__(self):
        super().__init__("csv_mapviz_player")

        with open("output/FM_1362_to_FM_2000.json", "r") as f:
            self.route_data = json.load(f)

        self.odom_df = pd.read_csv("combined_novatel_odom_data.csv")

        self.route_gdf, self.intersections_gdf, self.projected_crs = build_projected_route_and_intersections(self.route_data)

        self.origin_x = float(self.odom_df.iloc[0]["position_x"])
        self.origin_y = float(self.odom_df.iloc[0]["position_y"])

        self.path_pub = self.create_publisher(Path, "/osm_route_path", 10)
        self.odom_pub = self.create_publisher(Odometry, "/vehicle/odom", 10)
        self.marker_pub = self.create_publisher(MarkerArray, "/osm_route_markers", 10)
        self.alert_pub = self.create_publisher(String, "/driver_alert_text", 10)

        self.route_path = self.build_path_msg()
        self.marker_array = self.build_marker_array()


        self.index = 0

        self.create_timer(1.0, self.publish_static_layers)

    def build_path_msg(self):
        msg = Path()
        msg.header.frame_id = "map"

        for _, row in self.route_gdf.iterrows():
            line = row.geometry
            for x, y in line.coords:
                pose = PoseStamped()
                pose.header.frame_id = "map"
                pose.pose.position.x = float(x)-self.origin_x
                pose.pose.position.y = float(y)-self.origin_y
                pose.pose.position.z = 0.0
                pose.pose.orientation.w = 1.0
                msg.poses.append(pose)

        return msg

    def build_marker_array(self):
        marker_array = MarkerArray()

        for _, row in self.intersections_gdf.iterrows():
            marker = Marker()
            marker.header.frame_id = "map"
            marker.ns = "route_intersections"
            marker.id = int(row["idx"])
            marker.type = Marker.SPHERE
            marker.action = Marker.ADD
            marker.pose.position.x = float(row.geometry.x)-self.origin_x
            marker.pose.position.y = float(row.geometry.y)-self.origin_y
            marker.pose.position.z = 0.0
            marker.pose.orientation.w = 1.0
            marker.scale.x = 12.0
            marker.scale.y = 12.0
            marker.scale.z = 12.0

            if row["is_signalized"]:
                marker.color.r = 1.0
                marker.color.g = 1.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            elif row["is_stop_controlled"]:
                marker.color.r = 1.0
                marker.color.g = 0.0
                marker.color.b = 0.0
                marker.color.a = 1.0
            else:
                marker.color.r = 0.0
                marker.color.g = 0.5
                marker.color.b = 1.0
                marker.color.a = 1.0

            marker_array.markers.append(marker)

        return marker_array

    def publish_static_layers(self):
        now = self.get_clock().now().to_msg()

        self.route_path.header.stamp = now
        for pose in self.route_path.poses:
            pose.header.stamp = now

        for marker in self.marker_array.markers:
            marker.header.stamp = now

        self.path_pub.publish(self.route_path)
        self.marker_pub.publish(self.marker_array)

    def publish_vehicle(self):
        if self.index >= len(self.odom_df):
            return

        row = self.odom_df.iloc[self.index]

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.child_frame_id = "base_link"
        msg.pose.pose.position.x = float(row["position_x"])-self.origin_x
        msg.pose.pose.position.y = float(row["position_y"])-self.origin_y
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
        while rclpy.ok() and node.index < len(node.odom_df):
            node.publish_vehicle()
            rclpy.spin_once(node, timeout_sec=0.0)
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
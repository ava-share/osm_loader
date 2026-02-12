#!/usr/bin/env python3
"""ROS 2 node for downloading OpenStreetMap extracts via Overpass API.

This node supports three modes:
1. 'current': Downloads map based on robot's current pose from /current_pose topic
2. 'place': Downloads map for a geocoded location string
3. 'file': Copies an existing OSM file to the output directory

Optionally launches a visualization window showing the downloaded map with detailed
road network information including speed limits, lanes, traffic signals, and more.

Parameters:
    mode (str): Download mode - 'current', 'place', or 'file' (default: 'current')
    location_string (str): Location to geocode (for 'place' mode)
    osm_path (str): Path to existing OSM file (for 'file' mode)
    output_dir (str): Directory to save maps (default: ~/maps)
    visualize (bool): Auto-launch visualizer after download (default: true)

Example:
    ros2 run map_fetcher map_fetcher_node --ros-args -p mode:=place \
        -p location_string:="College Station, TX, USA"
"""
from __future__ import annotations

import math
from pathlib import Path
import subprocess
from typing import Optional

import requests
from geopy.geocoders import Nominatim
from geometry_msgs.msg import PoseStamped
import rclpy
from rclpy import init, shutdown
from rclpy.node import Node
from std_msgs.msg import String


class MapFetcherNode(Node):
    def __init__(self) -> None:
        super().__init__("map_fetcher_node")
        self.declare_parameter("mode", "current")  # current/place/file
        self.declare_parameter("location_string", "")
        self.declare_parameter("osm_path", "")
        self.declare_parameter("output_dir", str(Path.home() / "maps"))
        self.declare_parameter("visualize", True)
        self.output_dir = Path(self.get_parameter("output_dir").get_parameter_value().string_value)
        self.mode = self.get_parameter("mode").get_parameter_value().string_value
        self.location_string = self.get_parameter("location_string").get_parameter_value().string_value
        self.osm_path = Path(self.get_parameter("osm_path").get_parameter_value().string_value or "")
        self.output_dir.mkdir(parents=True, exist_ok=True)

        self.publisher = self.create_publisher(String, "map_fetcher/status", 10)
        self.pose_sub = self.create_subscription(PoseStamped, "current_pose", self.pose_callback, 10)
        self.geolocator = Nominatim(user_agent="ros2_map_fetcher")
        self.latest_pose: Optional[PoseStamped] = None
        self.completed = False
        self.timer = self.create_timer(3.0, self.timer_callback)

    def timer_callback(self) -> None:
        if self.completed:
            return
        if self.mode == "current":
            if not self.latest_pose:
                self.publisher.publish(String(data="waiting for pose..."))
                return
            lat = self.latest_pose.pose.position.y
            lon = self.latest_pose.pose.position.x
            filename = "current_location.osm"
            if self.fetch_and_store(lat, lon, filename):
                if self.get_parameter("visualize").get_parameter_value().bool_value:
                    self.launch_visualizer(self.output_dir / filename)
                self.completed = True
                self.timer.cancel()
        elif self.mode == "place":
            if not self.location_string:
                self.publisher.publish(String(data="location_string parameter is empty"))
                return
            lat, lon = self.geocode(self.location_string)
            if lat and lon:
                filename = f"{self.location_string.replace(' ', '_')}.osm"
                if self.fetch_and_store(lat, lon, filename):
                    if self.get_parameter("visualize").get_parameter_value().bool_value:
                        self.launch_visualizer(self.output_dir / filename)
                    self.completed = True
                    self.timer.cancel()
        elif self.mode == "file":
            if not self.osm_path.exists():
                self.publisher.publish(String(data=f"File {self.osm_path} not found"))
                return
            target = self.output_dir / self.osm_path.name
            target.write_bytes(self.osm_path.read_bytes())
            self.publisher.publish(String(data=f"Copied {self.osm_path} to {target}"))
            if self.get_parameter("visualize").get_parameter_value().bool_value:
                self.launch_visualizer(target)
            self.completed = True
            self.timer.cancel()
        else:
            self.publisher.publish(String(data=f"Unsupported mode {self.mode}"))

    def pose_callback(self, msg: PoseStamped) -> None:
        self.latest_pose = msg

    def geocode(self, text: str) -> tuple[Optional[float], Optional[float]]:
        location = self.geolocator.geocode(text, exactly_one=True, timeout=10)
        if not location:
            self.publisher.publish(String(data=f"Failed to geocode {text}"))
            return None, None
        return location.latitude, location.longitude

    def fetch_and_store(self, lat: float, lon: float, filename: str) -> bool:
        miles = 1.0
        meters = miles * 1609.344
        deg_lat = meters / 111_320
        deg_lon = meters / (111_320 * math.cos(math.radians(lat)))
        min_lat, max_lat = lat - deg_lat, lat + deg_lat
        min_lon, max_lon = lon - deg_lon, lon + deg_lon
        query = (
            f"[out:xml][timeout:25];"
            f"("
            f"  node({min_lat},{min_lon},{max_lat},{max_lon});"
            f"  way({min_lat},{min_lon},{max_lat},{max_lon});"
            f"  relation({min_lat},{min_lon},{max_lat},{max_lon});"
            f");"
            f"out body;"
            f">;"
            f"out skel qt;"
        )
        resp = requests.post(
            "https://overpass-api.de/api/interpreter",
            data={"data": query},
            timeout=60,
        )
        if resp.status_code != 200:
            self.publisher.publish(String(data=f"Overpass error {resp.status_code}"))
            return False
        dest = self.output_dir / filename
        dest.write_text(resp.text)
        self.publisher.publish(String(data=f"Saved map to {dest}"))
        return True

    def launch_visualizer(self, osm_file: Path) -> None:
        """Launch the visualizer node in a separate process."""
        self.get_logger().info('Launching visualizer...')
        subprocess.Popen([
            'ros2', 'run', 'map_fetcher', 'osm_visualizer_node',
            '--ros-args', '-p', f'osm_file:={osm_file}'
        ])


def main(args=None) -> None:
    init(args=args)
    node = MapFetcherNode()
    while rclpy.ok() and not node.completed:
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_node()
    shutdown()


if __name__ == "__main__":
    main()

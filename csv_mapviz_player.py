#!/usr/bin/env python3

import pandas as pd
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import NavSatFix, NavSatStatus
from std_msgs.msg import Int32
from pyproj import Transformer


class CsvMapvizPlayer(Node):
    def __init__(self):
        super().__init__("csv_mapviz_player")

        self.declare_parameter("csv_file", "combined_novatel_odom_data.csv")
        self.declare_parameter("utm_epsg", 32614)
        self.declare_parameter("publish_hz", 10.0)

        csv_file = self.get_parameter("csv_file").value
        utm_epsg = int(self.get_parameter("utm_epsg").value)
        publish_hz = float(self.get_parameter("publish_hz").value)

        self.df = pd.read_csv(csv_file)

        required_cols = [
            "timestamp",
            "position_x",
            "position_y",
            "position_z",
            "orientation_x",
            "orientation_y",
            "orientation_z",
            "orientation_w",
        ]
        for col in required_cols:
            if col not in self.df.columns:
                raise RuntimeError(f"CSV is missing required column: {col}")

        self.transformer = Transformer.from_crs(f"EPSG:{utm_epsg}", "EPSG:4326", always_xy=True)

        self.fix_pub = self.create_publisher(NavSatFix, "/gps/fix", 10)
        self.row_pub = self.create_publisher(Int32, "/csv_row_index", 10)

        origin_qos = QoSProfile(
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
        )
        self.origin_pub = self.create_publisher(NavSatFix, "/gps/fix/origin", origin_qos)

        self.index = 0
        self.origin_published = False

        self.timer = self.create_timer(1.0 / publish_hz, self.timer_callback)

        self.get_logger().info(f"Loaded CSV: {csv_file}")
        self.get_logger().info(f"Rows: {len(self.df)}")
        self.get_logger().info(f"Using UTM EPSG:{utm_epsg}")

    def build_fix(self, lat, lon):
        msg = NavSatFix()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "gps"
        msg.status.status = NavSatStatus.STATUS_FIX
        msg.status.service = NavSatStatus.SERVICE_GPS
        msg.latitude = lat
        msg.longitude = lon
        msg.altitude = 0.0
        return msg

    def timer_callback(self):
        if self.index >= len(self.df):
            self.get_logger().info("CSV playback complete.")
            self.timer.cancel()
            return

        row = self.df.iloc[self.index]

        x = float(row["position_x"])
        y = float(row["position_y"])

        lon, lat = self.transformer.transform(x, y)

        fix_msg = self.build_fix(lat, lon)
        self.fix_pub.publish(fix_msg)

        if not self.origin_published:
            self.origin_pub.publish(fix_msg)
            self.origin_published = True
            self.get_logger().info(f"Published origin lat={lat:.8f}, lon={lon:.8f}")

        row_msg = Int32()
        row_msg.data = int(self.index)
        self.row_pub.publish(row_msg)

        self.index += 1


def main(args=None):
    rclpy.init(args=args)
    node = CsvMapvizPlayer()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
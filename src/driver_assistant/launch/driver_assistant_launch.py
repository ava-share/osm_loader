from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    route_file_arg = DeclareLaunchArgument(
        "route_file",
        default_value="/home/avalocal/Downloads/osm_loader/output/FM_1362_to_FM_2000.json",
        description="Absolute path to the OSM route JSON file",
    )
    odom_csv_arg = DeclareLaunchArgument(
        "odom_csv",
        default_value="/home/avalocal/Downloads/osm_loader/combined_novatel_odom_data.csv",
        description="Absolute path to the odometry CSV file",
    )
    publish_rate_arg = DeclareLaunchArgument(
        "publish_rate_hz",
        default_value="10.0",
        description="CSV playback publish rate in Hz (csv_mapviz_player only)",
    )

    route_file = LaunchConfiguration("route_file")
    odom_csv = LaunchConfiguration("odom_csv")
    publish_rate = LaunchConfiguration("publish_rate_hz")

    driver_assistant = Node(
        package="driver_assistant",
        executable="driver_assistant_node",
        name="driver_assistant_node",
        output="screen",
        parameters=[
            {"route_file": route_file},
            {"odom_csv": odom_csv},
            {"lookahead_m": 150.0},
            {"min_progress_m": 2.0},
        ],
    )

    csv_mapviz_player = Node(
        package="driver_assistant",
        executable="csv_mapviz_player",
        name="csv_mapviz_player",
        output="screen",
        parameters=[
            {"route_file": route_file},
            {"odom_csv": odom_csv},
            {"publish_rate_hz": publish_rate},
        ],
    )

    return LaunchDescription([
        route_file_arg,
        odom_csv_arg,
        publish_rate_arg,
        driver_assistant,
        csv_mapviz_player,
    ])

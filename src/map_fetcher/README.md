# map_fetcher

This ROS 2 Python package exposes a single executable, `map_fetcher_node`, that downloads OpenStreetMap extracts based on the current pose, a configured place, or an existing OSM file provided via parameters.

## Installation
1. Source your ROS 2 distribution and create a workspace if needed. This repository already uses `src/` as the workspace source directory.
2. From the workspace root (`osm_testing`), run `colcon build` to install the package.

## Running the node
Use the provided entry point:

```
ros2 run map_fetcher map_fetcher_node
```

Adjust `--ros-args -p` parameters such as `mode`, `location_string`, or `osm_path` to drive behavior.

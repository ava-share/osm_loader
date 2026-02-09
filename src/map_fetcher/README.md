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

### Mode options
`mode` controls how the map is selected:

- `current` (default): uses the latest `PoseStamped` on `current_pose`.
	- Parameters used: `current_pose` topic and `output_dir`.
- `place`: geocodes the `location_string` and downloads the map around it.
	- Parameters used: `location_string` and `output_dir`.
- `file`: copies an existing `.osm` file into `output_dir`.
	- Parameters used: `osm_path` and `output_dir`.

### Examples
```
ros2 run map_fetcher map_fetcher_node --ros-args -p mode:=place -p location_string:="College Station, TX, USA"
```

Output is written to `~/maps` by default. Change it with `-p output_dir:=/path/to/maps`.

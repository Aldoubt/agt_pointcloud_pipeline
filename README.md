# agt_pointcloud_pipeline

Reusable ROS 2 LiDAR preprocessing, self-filtering, rosbag diagnostics and filter-tuning framework.

## Why this repository exists

Point-cloud filtering should not live inside a navigation monolith. This repository provides one shared filtering core for:

- online navigation / mapping / localization preprocessing;
- offline rosbag analysis;
- RViz A/B validation of raw, rejected and filtered clouds;
- automated parameter analysis by scripts or AI agents.

The core rule is **one filter definition, multiple frontends**. Offline analysis and robot runtime must use the same geometry and parameter semantics.

## Packages

- `agt_pointcloud_core`: ROS-independent C++ filter primitives and pipeline types.
- `agt_pointcloud_pipeline`: ROS 2 Humble runtime, pluginlib filter interface, TF-aware point-cloud processing.
- `agt_pointcloud_tools`: offline rosbag utilities; v0.1 includes IMU-based static-segment detection.

## v0.1 runtime flow

```text
sensor driver / adapter
        |
        v
sensor_msgs/PointCloud2
        |
        v
agt_pointcloud_pipeline
  range -> self box -> rear sector -> ...
        |
        +--> raw
        +--> rejected
        '--> filtered
```

Filters are evaluated in a declared robot frame (normally `base_link`) but the output cloud keeps the input frame and original XYZ coordinates. TF is used only for geometric classification.

## Build

```bash
cd ~/ros2_ws
git clone https://github.com/Aldoubt/agt_pointcloud_pipeline.git src/agt_pointcloud_pipeline
source /opt/ros/humble/setup.bash
rosdep install --from-paths src/agt_pointcloud_pipeline/src --ignore-src -r -y
colcon build --symlink-install --packages-up-to agt_pointcloud_pipeline agt_pointcloud_tools
```

## Run

```bash
source install/setup.bash
ros2 launch agt_pointcloud_pipeline filter.launch.py profile:=navigation
```

Default debug topics:

- `/agt/pointcloud/raw`
- `/agt/pointcloud/rejected`
- `/agt/pointcloud/filtered`

## Offline static detection

```bash
agt-lidar-analyze \
  --bag /path/to/rosbag \
  --imu-topic /agt/sensors/imu/data \
  --output /tmp/static_segments.yaml
```

This first milestone deliberately keeps the analyzer deterministic. The next layer will accumulate static point clouds, calculate polar persistence, and provide an interactive tuner that exports the same ROS profile YAML used at runtime.

See `docs/ARCHITECTURE.md`, `docs/PLUGIN_API.md`, and `docs/INTEGRATION_V3.md`.

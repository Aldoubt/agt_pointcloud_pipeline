# agt_pointcloud_pipeline

Reusable ROS 2 LiDAR preprocessing, self-filtering, rosbag diagnostics and filter-tuning framework.

## Scope

One shared filtering core serves:

- online navigation / mapping / localization preprocessing;
- offline rosbag analysis;
- RViz A/B validation of raw, rejected and filtered clouds;
- automated parameter analysis by scripts or AI agents.

The core rule is **one filter definition, multiple frontends**.

## Packages

- `agt_pointcloud_core`: ROS-independent C++ filter primitives.
- `agt_pointcloud_pipeline`: ROS 2 Humble runtime + pluginlib filters.
- `agt_pointcloud_tools`: rosbag IMU/static detection and polar-persistence analysis.

## Runtime

```text
PointCloud2
    |
    v
TF classification in robot frame
    |
range -> self box -> rear sector -> ...
    |
    +--> /agt/pointcloud/raw
    +--> /agt/pointcloud/rejected
    '--> /agt/pointcloud/filtered
```

Run:

```bash
ros2 launch agt_pointcloud_pipeline filter.launch.py profile:=navigation
```

Profiles:

- `navigation`: bounded rear mask allowed;
- `mapping`: conservative, no default rear-sector deletion;
- `localization`: preserve scan-matching features;
- `analysis`: debug outputs enabled.

## Automatic rosbag analysis

The analyzer first detects stationary IMU intervals, then accumulates LiDAR observations over the selected interval and calculates an angle/range persistence map.

```bash
agt-lidar-analyze \
  --bag /path/to/rosbag \
  --imu-topic /agt/sensors/imu/data \
  --cloud-topic auto \
  --extrinsic-yaml /path/to/source_to_robot.yaml \
  --output /tmp/lidar_analysis.yaml
```

Outputs:

- static intervals and selected interval;
- polar persistence CSV;
- bounded rear-sector recommendation;
- runtime-compatible parameter snippet.

For clouds already expressed in the robot frame, use `--assume-source-is-robot-frame` explicitly.

See `docs/ROSBAG_ANALYSIS.md`, `docs/ARCHITECTURE.md`, `docs/PLUGIN_API.md`, and `docs/INTEGRATION_V3.md`.

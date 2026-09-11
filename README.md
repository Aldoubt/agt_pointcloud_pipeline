# agt_pointcloud_pipeline

Reusable ROS 2 LiDAR preprocessing, self-filtering, rosbag diagnostics and filter-tuning framework.

## Scope

One shared filtering core serves:

- online navigation / mapping / localization preprocessing;
- offline rosbag analysis;
- visual parameter tuning;
- RViz A/B validation of raw, rejected and filtered clouds;
- automated parameter analysis by scripts or AI agents.

The core rule is **one filter definition, multiple frontends**.

## Packages

- `agt_pointcloud_core`: ROS-independent C++ filter primitives.
- `agt_pointcloud_pipeline`: ROS 2 Humble runtime + pluginlib filters.
- `agt_pointcloud_tools`: rosbag static detection, polar persistence analysis and tuner UI.

## 1. Automatic rosbag analysis

```bash
agt-lidar-analyze \
  --bag /path/to/rosbag \
  --imu-topic /agt/sensors/imu/data \
  --cloud-topic auto \
  --extrinsic-yaml /path/to/source_to_robot.yaml \
  --output /tmp/lidar_analysis.yaml
```

Outputs:

- detected static intervals;
- selected static interval;
- polar persistence CSV;
- bounded rear-sector recommendation;
- runtime parameter snippet.

## 2. Visual tuning

```bash
agt-lidar-tuner \
  --csv /tmp/lidar_analysis_polar.csv \
  --suggestion /tmp/lidar_analysis_suggested_filter.yaml \
  --output /tmp/tuned_filter.yaml
```

The GUI provides a robot-centric top view and editable angle/range/Z bounds.

## 3. Full-bag RViz validation

```bash
ros2 launch agt_pointcloud_pipeline filter_debug.launch.py profile:=analysis
```

Then replay the bag. RViz shows:

- Raw: gray;
- Rejected: red;
- Filtered: green.

Only parameters that survive the complete motion replay should be promoted to a navigation profile.

Profiles:

- `navigation`: bounded rear mask allowed;
- `mapping`: conservative;
- `localization`: preserve scan-matching features;
- `analysis`: debug outputs enabled.

See `docs/ROSBAG_ANALYSIS.md`, `docs/TUNER.md`, `docs/ARCHITECTURE.md`, `docs/PLUGIN_API.md`, and `docs/INTEGRATION_V3.md`.

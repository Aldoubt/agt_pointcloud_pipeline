# Rosbag filter analysis

## Purpose

Automatically find a stationary interval and use repeated LiDAR observations to identify persistent near-field robot/self-interference candidates.

The analyzer is intentionally conservative. It does **not** directly modify a navigation profile.

## Supported cloud messages

- `sensor_msgs/msg/PointCloud2`
- `livox_ros_driver2/msg/CustomMsg`

Cloud topic `auto` prefers:

1. `/agt/livox/points`
2. `/fastlio2/body_cloud`
3. `/agt/sensors/lidar/custom`
4. `/livox/lidar`

## Coordinate frame requirement

Polar sectors are meaningful only in a robot/body frame.

For a tilted MID360, provide a measured source-to-robot transform:

```yaml
source_to_robot:
  translation: [x, y, z]
  rpy_deg: [roll, pitch, yaw]
```

Run:

```bash
agt-lidar-analyze \
  --bag /path/to/bag \
  --imu-topic /agt/sensors/imu/data \
  --cloud-topic auto \
  --extrinsic-yaml /path/to/source_to_robot.yaml \
  --output /tmp/lidar_analysis.yaml
```

If the selected cloud is already expressed in the intended robot frame, identity must be explicit with `--assume-source-is-robot-frame`.

## Outputs

For `lidar_analysis.yaml`, the analyzer also writes:

- `lidar_analysis_polar.csv`: angle/range persistence table;
- `lidar_analysis_suggested_filter.yaml`: runtime parameter snippet.

Persistence for a polar cell is:

```text
frames containing the cell / analyzed frames
```

## RViz full-bag A/B validation

After copying the suggested values, launch:

```bash
ros2 launch agt_pointcloud_pipeline filter_debug.launch.py \
  profile:=analysis \
  input_topic:=/agt/livox/points \
  target_frame:=base_link \
  rear_center_deg:=180.0 \
  rear_width_deg:=20.0 \
  rear_min_range_m:=0.3 \
  rear_max_range_m:=1.2
```

Then replay the whole bag in another terminal:

```bash
ros2 bag play /path/to/bag
```

RViz displays:

- Raw: gray;
- Rejected: red;
- Filtered: green.

The runtime logs cumulative input/output/rejected counts and per-filter rejection counts once per second in debug mode.

For a raw Livox `CustomMsg` bag, run the project's existing Livox-to-PointCloud2 adapter before this runtime check. The offline analyzer itself can read CustomMsg directly.

## Acceptance

Inspect the entire motion sequence, not only the stationary calibration segment:

- turns;
- slopes;
- reversing;
- close walls;
- real rear obstacles;
- people behind the robot.

Only promote a sector to the navigation profile when self-interference is removed without hiding useful environment geometry.

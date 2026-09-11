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

If the selected cloud is already expressed in the intended robot frame, identity must be explicit:

```bash
agt-lidar-analyze ... --assume-source-is-robot-frame
```

The tool refuses to silently treat a tilted LiDAR frame as `base_link`.

## Outputs

For `lidar_analysis.yaml`, the analyzer also writes:

- `lidar_analysis_polar.csv`: angle/range persistence table;
- `lidar_analysis_suggested_filter.yaml`: runtime parameter snippet.

Persistence for a polar cell is:

```text
frames containing the cell / analyzed frames
```

Only a bounded rear search window and bounded range/Z region are analyzed by default. This avoids recommending an infinite rear dead zone.

## Recommended validation

1. inspect the suggested sector;
2. copy the snippet into the analysis profile;
3. launch the debug runtime;
4. replay the **entire** bag;
5. inspect raw/rejected/filtered clouds;
6. verify turns, slopes, reversing, walls and real rear obstacles;
7. only then promote the parameters to navigation.

# Self geometry and dynamic-interference analysis

The offline analyzer separates three concepts that must not be conflated:

1. **self geometry**: points that are persistent in the robot frame and lie inside the robot geometry envelope;
2. **external persistent structure**: persistent robot-frame observations outside the robot envelope; these may be walls, vegetation, equipment or a person following the robot;
3. **dynamic world objects**: objects whose map-frame support changes over time.

A high robot-frame persistence score alone is not enough to declare a point as self geometry.

## Cross-segment consensus

Use several independently detected stationary intervals from the same bag:

```bash
ros2 run agt_pointcloud_tools agt-lidar-consensus \
  --bag /path/to/bag \
  --imu-topic /agt/sensors/imu/data \
  --cloud-topic /agt/sensors/lidar/custom \
  --extrinsic-yaml /path/to/source_to_robot.yaml \
  --point-stride 1 \
  --max-frames-per-segment 100 \
  --angle-bin-deg 2 \
  --range-bin-m 0.05 \
  --analysis-min-range-m 0.05 \
  --analysis-max-range-m 1.5 \
  --within-segment-persistence-min 0.5 \
  --segment-support-min 0.6 \
  --robot-length-m 1.023 \
  --robot-width-m 0.778 \
  --output /tmp/self_consensus.yaml
```

The command reads the cloud stream once and distributes frames into all selected
static intervals. It produces `*_cells.csv` with `self_geometry` and
`external_persistent` labels.

## Following people

A person walking with the robot can stay persistent in the robot frame, so
robot-frame persistence cannot classify that person as dynamic. The next
classifier stage must use motion intervals and map-frame consistency:

- self geometry: inside robot envelope, very low angle/range variance;
- follower: outside robot envelope, robot-frame position roughly stable but with
  measurable angle/range/Z variation;
- static world: map-frame position stable while robot-frame angle/range changes.

Do not feed the external-persistent region directly into a permanent sector mask.

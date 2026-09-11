# Architecture

## Design goal

Provide one reusable point-cloud preprocessing domain that can be embedded in navigation systems or used independently for sensor analysis.

```text
                           agt_pointcloud_core
                         (no ROS dependencies)
                         /        |          \
                        /         |           \
              runtime plugins   analyzer      future GUI
                    |               |             |
               ROS2 node        rosbag2_py      human tuning
                    \               |             /
                     \--------------+------------/
                              same config semantics
```

## Packages

### agt_pointcloud_core

Owns:
- PointXYZ / PointContext;
- RangeFilter;
- BoxFilter;
- SectorFilter;
- FilterDecision.

It must not know about PointCloud2, TF, rosbag2, RViz, Livox, FAST-LIO2 or Nav2.

### agt_pointcloud_pipeline

Owns:
- PointCloud2 adapter;
- TF lookup from source cloud frame to `target_frame`;
- pluginlib lifecycle;
- raw/rejected/filtered publishers;
- per-filter counters.

Output points remain in the source frame. This avoids surprising downstream geometry changes. TF-transformed robot-frame points exist only in the evaluation context.

### agt_pointcloud_tools

Owns offline analysis:
- rosbag topic discovery;
- IMU static-segment detection;
- machine-readable reports.

Planned next:
- static cloud accumulation;
- polar angle/range density;
- voxel persistence;
- automatic self-interference candidate generation;
- interactive filter tuner;
- full-bag replay scoring.

## Profiles

Different consumers have different information-loss budgets.

- navigation: self geometry + bounded rear mask allowed;
- mapping: conservative, preserve environmental structure;
- localization: very conservative, preserve matching features;
- analysis: expose candidates and rejected points for inspection.

## Static-segment workflow

```text
rosbag IMU
   |
sliding windows
   |
gyro norm + gyro variance + accel norm variance
   |
merge adjacent windows
   |
static_segments.yaml
   |
(static cloud accumulation - next milestone)
```

Static detection is a sample-selection mechanism only. Runtime filtering is purely geometric and runs on every cloud.

## Safety rule for rear masks

Never use an unbounded rear sector by default. A rear sector should include:
- center angle;
- angular width;
- minimum range;
- maximum range;
- optional Z bounds.

This preserves useful environmental geometry behind the robot.

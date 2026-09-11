# AGENTS.md

## Scope

This repository owns reusable point-cloud preprocessing and diagnostics. It must not own navigation, localization state machines, map management, mission logic, or sensor-driver-specific behavior.

## Architectural constraints

1. `agt_pointcloud_core` must remain ROS-independent.
2. MID360 / Livox message conversion belongs in an adapter outside the core. Runtime input here is `sensor_msgs/PointCloud2`.
3. Geometry filters use robot-frame coordinates, normally `base_link`.
4. Filtering must never silently become the authoritative TF publisher.
5. Mapping, localization and navigation may use different profiles.
6. Aggressive rear-sector filtering must be range-bounded and disabled by default for localization.
7. Offline analyzer recommendations are suggestions until replay/A-B validation passes.
8. Any filter added to runtime should be reusable by offline analysis, or explicitly document why not.

## Development

Target: Ubuntu 22.04, ROS 2 Humble, C++17, Python 3.

Before merging:
- build all packages with colcon;
- run core tests;
- replay at least one rosbag;
- inspect raw/rejected/filtered clouds;
- document parameter changes.

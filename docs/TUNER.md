# Interactive LiDAR filter tuner

`agt-lidar-tuner` is the human-facing companion to `agt-lidar-analyze`.

The analyzer determines stable intervals and generates a polar persistence table. The tuner opens that table and lets the operator adjust a bounded exclusion sector before full 3D replay validation.

## Start

```bash
agt-lidar-tuner \
  --csv /tmp/lidar_analysis_polar.csv \
  --suggestion /tmp/lidar_analysis_suggested_filter.yaml \
  --output /tmp/tuned_filter.yaml
```

## View

The canvas is a top-down polar view in robot coordinates:

- center dot: robot origin;
- right: front / 0 degrees;
- left: rear / 180 degrees;
- yellow/orange cells: repeated polar observations, brighter means higher persistence;
- red boundary: current exclusion sector.

Controls:

- center angle;
- width;
- minimum/maximum planar range;
- minimum/maximum Z.

The Z controls also affect which persistence cells are displayed.

## Output

The saved YAML uses the same ROS parameter names as the runtime `SectorFilterPlugin`.

The GUI is intentionally not the final acceptance tool. After tuning, use `filter_debug.launch.py` and replay the complete bag. The 2D persistence view answers **where a repeated self-interference region is**; RViz answers **what real geometry would be lost while the robot moves**.

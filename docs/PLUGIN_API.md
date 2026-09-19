# Plugin API

Runtime filters implement `agt_pointcloud_pipeline::FilterPlugin`.

Each plugin receives a `PointContext`:

- `source`: point in the incoming PointCloud2 frame;
- `robot`: the same point transformed into `target_frame`.

A plugin returns `true` when the point should be rejected.

The runtime executes plugins in configured order and stops at the first rejection. This gives deterministic accounting: one rejected point belongs to exactly one filter counter.

## Filter chain

A runtime profile lists **instance names**:

```yaml
filter_chain: [range, self_box, rear_sector]
```

Each instance chooses a plugin class:

```yaml
filters.range.type: agt_pointcloud_pipeline/RangeFilterPlugin
filters.self_box.type: agt_pointcloud_pipeline/BoxFilterPlugin
filters.rear_sector.type: agt_pointcloud_pipeline/SectorFilterPlugin
```

Plugin parameters live below the same instance prefix.

This means one plugin type can be instantiated multiple times:

```yaml
filter_chain: [range, chassis, rear_left_pole, rear_right_pole]

filters.chassis.type: agt_pointcloud_pipeline/BoxFilterPlugin
filters.rear_left_pole.type: agt_pointcloud_pipeline/BoxFilterPlugin
filters.rear_right_pole.type: agt_pointcloud_pipeline/BoxFilterPlugin
```

The runtime does not need new C++ branches for each robot geometry primitive.

## Built-ins

### RangeFilterPlugin

Uses Euclidean distance in the incoming/source frame.

### BoxFilterPlugin

Axis-aligned exclusion box in `target_frame`. Use multiple instances for chassis, rods, tracks or mounts.

### SectorFilterPlugin

Angular exclusion sector in `target_frame`, bounded by planar range and Z. It is intended for measured blind/self-interference zones, not an infinite environmental rear mask.

## Extension rule

A new plugin should:
- keep deterministic semantics;
- avoid publishing TF;
- expose configuration through its instance prefix;
- include a core/unit test where possible;
- document which profiles should use it.

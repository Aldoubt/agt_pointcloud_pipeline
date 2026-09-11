# Plugin API

Runtime filters implement `agt_pointcloud_pipeline::FilterPlugin`.

Each plugin receives a `PointContext`:

- `source`: point in the incoming PointCloud2 frame;
- `robot`: the same point transformed into `target_frame`.

A plugin returns `true` when the point should be rejected.

The runtime executes plugins in configured order and stops at the first rejection. This gives deterministic accounting: one rejected point belongs to exactly one filter counter.

## Parameter namespace

A pipeline lists filter instance names:

```yaml
filters: [range, self_box, rear_sector]
```

Each instance has a type:

```yaml
filters.range.type: agt_pointcloud_pipeline/RangeFilterPlugin
filters.self_box.type: agt_pointcloud_pipeline/BoxFilterPlugin
filters.rear_sector.type: agt_pointcloud_pipeline/SectorFilterPlugin
```

Plugin parameters live below the same prefix.

This allows multiple box or sector instances without changing code.

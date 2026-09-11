# Integration with agt_navigation_v3

The navigation repository should consume this project as a dependency instead of keeping a second filtering implementation.

Recommended migration:

1. Keep the current v3 implementation unchanged while this repository passes rosbag replay acceptance.
2. Add this repository to the workspace/vcs manifest.
3. Start `agt_pointcloud_pipeline` from v3 bringup using the navigation profile.
4. Compare old/new:
   - output rate;
   - removed-point counts;
   - rear-rod removal;
   - preservation of real rear environment;
   - CPU usage.
5. Switch consumers to `/agt/pointcloud/filtered`.
6. Only then remove duplicated v3 filter code.

Do not feed an aggressively filtered navigation cloud back into FAST-LIO2 or Batch-LIO unless an explicit LIO experiment proves it is safe.

Suggested ownership after migration:

```text
agt_navigation_v3             agt_pointcloud_pipeline
-----------------             -----------------------
bringup/profile selection --> runtime node
Nav2 consumers             --> filtered output
mapping policy             --> chooses mapping profile
localization policy        --> chooses localization profile
                              filter algorithms
                              rosbag analysis
                              filter diagnostics
```

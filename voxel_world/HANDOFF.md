# 3D Voxel World Prototype Handoff

## Purpose and scope

This prototype builds a static 3D voxel representation of the `excavation_long`
Gazebo collision world and checks ray predictions against the simulated
Mid-360 point cloud. It is the first environment-validation step for the
proposed learned exploration planner.

The current planning scope is waypoint-only: the future policy should select
the next waypoint, with no heading choice as an action for now. This package
does not implement the policy, roadmap, reward, or training loop.

## Repository findings

- The `map_manager` package already maintains a 3D occupancy map and SDF-based
  collision information. The excavation scoring configuration uses a 0.2 m
  voxel resolution, bounds of 46 x 90 x 15 m, and origin `[-23, -45, -10.5]`.
- `uav_simulator/models/excavation_long/excavation_long.sdf` describes the
  static pit with box collision geometry. The voxel prototype reads those
  collisions rather than duplicating pit dimensions in source code.
- The simulated Livox plugin publishes `/cloud_registered_body` and the
  vehicle odometry is `/CERLAB/quadcopter/odom`. The sensor has a 0.13 m
  vertical offset and the launch accepts a LiDAR pitch parameter (23.5 degrees
  by default for the tested setup).
- The Livox plugin reads an 800,000-row recorded pattern. Each 10 Hz scan
  consumes 20,000 pattern rows with a downsample factor of 2, producing 10,000
  points per cloud. The plugin advances through 40 pattern frames. Comparing
  each cloud against frame zero gives incorrect range results.
- Existing repository changes were present in the `autonomous_flight`,
  `mission_control`, and `uav_simulator` areas. This work added only the new
  `voxel_world/` package and did not reset or edit those existing changes.

## Files added

- `package.xml` — ROS package metadata and runtime/build dependencies.
- `CMakeLists.txt` — catkin Python package and script installation setup.
- `setup.py` — makes the `voxel_world` Python module installable through
  catkin.
- `src/voxel_world/__init__.py` — public package exports.
- `src/voxel_world/world.py` — SDF box parsing, dense occupancy grid creation,
  box rasterization, Amanatides-Woo voxel ray traversal, Mid-360 CSV direction
  loading, quaternion conversion, and sensor pose helpers.
- `scripts/voxelize_sdf.py` — command-line conversion of an SDF model to a
  compressed NPZ voxel map.
- `scripts/compare_gazebo_scan.py` — ROS node that synchronizes point-cloud and
  odometry messages, predicts voxel-map ranges, and reports hit/miss overlap
  and range errors. It also writes per-ray results to CSV when requested.
- `README.md` — build and run instructions, supported geometry, and validation
  notes.
- `HANDOFF.md` — this implementation and validation summary.

## Voxel representation

`VoxelWorld.occupied` is a dense `uint8` array indexed `[z, y, x]`. Its default
dimensions are `(75, 450, 230)` at 0.2 m resolution, or 7,762,500 bytes before
compression. A voxel is marked occupied when its center is inside an SDF box.
Raycasting traverses cells and returns the distance to the first occupied
cell, or no hit when the ray exits the bounded grid without intersecting an
occupied cell.

The SDF reader currently supports box collision geometry and ordinary model,
link, and collision poses. It deliberately errors on unsupported collision
types, `relative_to` attributes, or included models rather than silently
creating an incomplete map. Mesh collision support is not implemented.

To generate the default map after sourcing the catkin workspace:

```bash
rosrun voxel_world voxelize_sdf.py --output /tmp/excavation_long_voxels.npz
```

## Gazebo comparison details

Run the existing `uav_simulator/start_excavation_long.launch` and then:

```bash
rosrun voxel_world compare_gazebo_scan.py \
  --scans 3 --rays 512 \
  --output-csv /tmp/voxel_gazebo_scan.csv
```

The comparator defaults to the same map bounds and 0.2 m resolution, 23.5
degree LiDAR pitch, 0.13 m sensor offset, and 20 m maximum range used in the
prototype setup. It selects the recorded-pattern frame from the synchronized
scan timestamp using a default 10 Hz sensor rate and a -1 scan index offset.
If the URDF update rate or plugin frame behavior changes, check those settings
against `uav_simulator/src/livox_lidar/livox_points_plugin.cpp` and adjust
`--scan-rate` or `--scan-index-offset` before comparing ranges.

`--rays 0` compares all 10,000 points in a scan. The output JSON reports the
selected pattern frame, hit counts, hit IoU, matched rays, mean absolute range
error, median absolute range error, and 95th percentile absolute range error.

## Validation performed

- Python source compilation completed successfully.
- Voxelization completed from the local SDF. Result: shape `(75, 450, 230)`,
  resolution 0.2 m, 131,370 occupied voxels, 7,762,500 dense bytes.
- Started the headless `excavation_long` Gazebo launch and compared three
  synchronized point-cloud/odometry scans at the drone's resulting pose.
- Per-scan results:

  | Scan | Pattern frame | Expected/Gazebo hits | Hit IoU | Range MAE | Median abs. error | P95 abs. error |
  | --- | ---: | ---: | ---: | ---: | ---: | ---: |
  | 0 | 37 | 483 / 483 | 1.000 | 0.213 m | 0.00062 m | 0.986 m |
  | 1 | 38 | 481 / 480 | 0.998 | 0.266 m | 0.00073 m | 1.446 m |
  | 2 | 39 | 487 / 486 | 0.998 | 0.219 m | 0.00010 m | 1.186 m |

Hit/miss agreement and median error are strong for these three scans. The
larger mean and 95th percentile errors show that a small number of rays still
have substantial range differences. The voxel model is therefore a good
initial geometry check, not a claim of exact scan equivalence. Likely follow-up
debugging should inspect those outlier ray indices, collision boundaries,
voxel traversal edge cases, and timestamp-to-pattern-frame alignment at other
poses. The tested drone had no position hold in this launch and settled near
the pit floor, so repeat comparisons at controlled poses before treating this
result as a whole-map validation.

The generated map and three-scan CSV were written outside the repository to
`/tmp/excavation_long_voxels.npz` and `/tmp/voxel_gazebo_scan_3.csv`.

## Suggested next implementation steps

1. Add a controllable set of static UAV poses and run comparisons at multiple
   heights and positions across the pit.
2. Investigate the high-error rays and confirm frame selection with a longer
   sequence that crosses the 40-frame pattern wrap.
3. Compare the voxel map's unknown/free/occupied conventions with
   `map_manager` if the voxel world will become the training observation rather
   than only simulator ground truth.
4. Add a compact 3D roadmap over the voxel world and implement waypoint-only
   selection. Keep heading out of the action space in this initial version.
5. Once single-step execution is stable, add exploration coverage state and
   reward/training code; retain baselines and per-stage diagnostics so
   navigation failures can be separated from voxel and sensor-model errors.

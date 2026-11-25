# Voxel world prototype

This package builds a dense ground-truth voxel grid from SDF collision boxes
and checks its ray predictions against the actual Gazebo Mid-360 point cloud.
The first target is the static `excavation_long` model. Its map bounds and
0.2 m cell size match `autonomous_flight/cfg/dynamic_exploration/maps/excavation_long_mapping.yaml`.

The implementation reads collision geometry from the SDF rather than copying
pit dimensions into Python. It currently supports box collisions and regular
SDF model/link/collision poses. Mesh collision geometry, included models,
`relative_to` frames, sensor noise, and motion during a scan are not modeled;
unsupported SDF geometry raises an error.

## Build a voxel map

After sourcing the workspace:

```bash
rosrun voxel_world voxelize_sdf.py --output /tmp/excavation_long_voxels.npz
```

The output grid is `occupied[z, y, x]`, with one byte per cell. The default
map contains about 7.8 million cells (roughly 7.8 MB uncompressed).

## Compare predicted rays with Gazebo

Terminal 1, start the static excavation world and Mid-360 at a safe pit pose:

```bash
roslaunch uav_simulator start_excavation_long.launch gui:=false keyboard:=false \
  spawn_x:=10.0 spawn_y:=8.0 spawn_z:=-8.5 spawn_yaw:=1.5708 lidar_pitch_deg:=23.5
```

Terminal 2, sample 2,048 of the ordered Livox rays from one synchronized scan:

```bash
rosrun voxel_world compare_gazebo_scan.py --scans 1 --output-csv /tmp/voxel_gazebo_scan.csv
```

The validator compares hit/miss agreement and range error up to the same 20 m
maximum range used by the mapper. Set `--rays 0` to compare every point in the
scan. The recorded Livox file contains 800,000 rays and Gazebo consumes 20,000
per scan, so the validator selects the matching one of 40 pattern frames from
the synchronized scan timestamp. `--scan-rate` defaults to the sensor's 10 Hz;
change it if the URDF update rate changes. The startup pose is inside the pit,
but this launch does not hold the vehicle at that pose, so capture soon after
startup or add a position hold when testing other parts of the map.

## Reading the comparison

The voxel model classifies a cell occupied when its center lies inside an SDF
collision box. That discretization can shift a predicted hit by up to about one
voxel at a surface. First inspect hit/miss IoU, then range MAE and the error
distribution. Large coherent errors suggest a frame, tilt, or ray-pattern
mismatch; small near-surface errors are expected from voxel quantization.

## Fast sensor and map for training (`fast.py`)

- `BoxScene`: ray hits against the SDF boxes themselves, all rays at once. Exact surfaces, so boxes thinner than
  a voxel never disappear (the dense grid above only marks a voxel when its centre is inside a box).
- `traverse`: the voxels each ray crosses, vectorized; checked exact against per-ray stepping.
- `OccupancyMap` and `PitScore`: map_manager's occMap update (log-odds, 20 m ray length, 10 m inspect range) and
  epic_monitor_node's coverage numbers (pit, block, floor, surface %).
- `mid360_rays`: the recorded Livox pattern, or an even grid over the same field of view (-7 to 52 deg).

Speed on this laptop: a 10,000-ray scan into the 0.2 m map takes about 120 ms; the training settings (0.4 m map,
1,500 even rays) about 11 ms per scan, ray-cast included.

## Replay a Gazebo flight (`replay_flight.py`)

Record a run with a bag of `/CERLAB/quadcopter/odom`, `/cloud_registered_body` and the mission state, then:

```bash
python3 scripts/replay_flight.py --bag run.bag --csv <monitor run CSV> --out replay/
```

It scores the flight three ways against the monitor's CSV: the recorded Gazebo scans into the Python map
(`gazebo`), the same rays re-cast against the boxes (`boxes`), and the training shortcuts (`training`), and writes
`replay_curves.csv`, `replay_summary.json` and `replay_curves.png`.

Result on excavation_long at 23.5 deg (2026-11-23): `boxes` stays within about 3 % of `gazebo` for pit, block and
floor % over the whole flight; `training` too after the first seconds. Both sit far above the monitor's pit and
block % because map_manager's own ray update clears much less free space than a full ray walk: it walks rays on a
grid that ignores the map origin (half a voxel off in z when ground_height is not a multiple of the resolution, as
in every pit config) and stops each ray at the first voxel an earlier ray crossed. `check_occmap_raycast.py`
reproduces the monitor's number with a line-by-line port and switches each quirk off. Floor % and surface % depend
on hits, not on cleared space, and match the monitor.

## Training worlds and evaluation courses

Generated sets are not kept in git: they come back exactly from the scripts below and a seed.

- `scripts/make_ari3d.py` (module `ari3d.py`): 3D training worlds from ARiADNE's map PNGs
  (`~/rl/large-scale-DRL-exploration/maps`). Each region gets a level (floor and ceiling move together), some are
  pits, islands may be low or have lips, and hurdle/lintel gates close any single flight height that reaches too
  much. Every opening is at least `passage` (3.2 m) and reachable with 0.5 m clearance. `accepted` in stats.json
  marks worlds that pass both tests. Needs scipy and scikit-image (`~/.venvs/ariadne/bin/python`).
  `make_ari3d.py --count 8 --seed 0 --out <dir>`; `--flat` gives the plain 2D map extruded.
- `scripts/make_courses.py` (module `course.py`): evaluation courses, closed rooms with obstacles at every height
  (`--layout room`, test seeds 0-99). `--layout maze` is an earlier training layout, kept for reference.
- `scripts/view_check.py`: how much surface the best 1-2 viewpoints see (line of sight in the lidar's band).
- `scripts/slice_check.py`: per-column height options ("slices") on the excavation worlds: node counts,
  connectivity and surfaces seen.
- `scripts/prune_study.py`: compares node-pruning strategies (full lattice, slices, utility gating, one height per
  column) with a greedy explorer on `make_ari3d` worlds.

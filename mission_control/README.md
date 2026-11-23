# Mission Control

One window for simulation and real flights. Controls are on the left, the live 3D view fills the rest, and the mission data sits in a bar along the bottom that opens into more detail.

```bash
./mission_control.sh          # opens the window (Chromium app mode, else the default browser)
```

Ctrl-C in that terminal stops the server and every simulation task cleanly. Drone tasks keep running on the drone.

## Using it
- **Simulation tab:**
  - **Simulator:** pick a world and a drone from the galleries, then press Start. The view connects by itself.
    - Every file in `uav_simulator/worlds` and `uav_simulator/urdf` appears; worlds with several variants show them as chips.
    - Before anything runs, the centre shows the chosen world as a wireframe and the drone where it will start.
    - It all goes through `uav_simulator/launch/start_world.launch world:=... drone:=...`.
  - **Mission:** choose Exploration, Navigation or Inspection, press Launch, then use Start, Pause, Home and Land.
    - Force quit kills the planner at once, without a clean shutdown.
  - **Recording:** each section has a record dot and a size choice (Small, Medium, Large); the list says what each includes.
- **Drone tab:** press Connect (SSH), start Localization and Sensors, then the mission as in the sim.
  - Drone tasks run in `tmux` sessions on the drone, so a WiFi drop never kills a flight.
  - When you reconnect, the window picks up whatever is already running.
- **Bottom bar:** state, time, explored / floor / surfaces, height, speed and path. Open it (`L`) for:
  - **Mission:** every status value.
  - **Topics:** all live topics; click one to see its messages and rate.
  - **Processes:** each process's output, plus **+ Terminal** for a real bash with the tab's ROS setup (on the drone tab, a shell on the drone).
  - **Storage:** disk and recordings.
- **Stop:** sends Ctrl-C, then SIGTERM after 15 s, then SIGKILL. Recordings close properly.
- **Show and hide:** `[` hides the controls. The view's top-right buttons switch map colours (Mono, Height, Camera), follow the drone, and toggle layers (world model, map, trail, path, sensor coverage). The window remembers what you left open.
- **Links:** `?tab=drone` and `?data=topics` (or `mission`, `procs`, `storage`) open the window on that tab.

## Setup
Everything is in `config.json`; restart the server after editing it.
- **Tasks:** each task has a `cmd`. Use `variants` for choices and `needs` to grey a button out until other tasks are running.
  - `hidden` tasks have no button. Live view and RViz are hidden.
  - `auto` tasks start with the first other task of their tab.
- **Worlds and drones:**
  - Drone names and notes go in `sim.drones`.
  - Start positions go in `sim.spawn`, by file or folder. A world file can also say `drone start (x, y, z)` in a comment.
  - The drone previews are drawn from each URDF's size and sensors until CAD models are added.
- **Drone buttons left empty:** **Localization** and **Sensors** have empty commands until you add the drone's launch files.
- **Drone setup:**
  - key login: `ssh-copy-id cerlab@192.168.0.252`;
  - `tmux` installed on the drone;
  - `rosbridge_server` installed on the drone for Live view.
- **ROS master:**
  - The sim tab always uses a local ROS master (`env` in the config), whatever `~/.bashrc` says. A drone-network `ROS_MASTER_URI` otherwise makes roslaunch hang off that network.
  - The drone tab uses the drone's master. For laptop RViz on the drone network, `ROS_IP` must be the laptop's address there.
- **Coloured map:** the Camera colour mode Camera colours come from `viz.colored_map` (default `/reconstruction/cloud`), a PointCloud2 with an `rgb` field; it shows as soon as something publishes it.

## Disk space
Everything lives in `~/mission_control_data`.

| What | Limit |
|---|---|
| Task logs | 5 MB per file (one rotation); 300 MB for all sessions, oldest session deleted first |
| ROS node logs (`ROS_LOG_DIR`) | In the session folder; any file over the cap is trimmed to its last half |
| Recordings | lz4 compressed (about half size), split into 1 GB pieces, so a crash loses at most one piece |
| Disk guard | A recording won't start with less than 5 GB + one piece free, and stops cleanly below 5 GB |
| Drone recordings | Saved on the drone, never sent over WiFi; the drone's free space is shown in Storage |

- **Size choices:** Small records pose, status and paths. Medium adds the map or LiDAR. Large adds cameras or planner internals. The MB/min figures in the list are rough estimates, not measurements.
- **Viewing and deleting:** Storage lists recordings; deleting asks for confirmation.

## Drones from CAD
Use the CAD environment: `~/drone_cad/.venv/bin/python`, which has cadquery-ocp, trimesh and fast-simplification.

1. **Export** the assembly from Onshape as **one** STEP file. Separate part files lose where each part sits.
2. **Read it:** `tools/cad_assembly.py "Assembly.step" parts.npz` meshes every part in place and prints a table of names, sizes and positions.
3. **Write a spec:** `uav_simulator/urdf/<drone>/model_spec.json` gives:
   - which parts form which group, and each group's triangle budget;
   - the body origin;
   - the hinge point and the tilt the CAD was drawn at;
   - where the LiDAR frame sits.
4. **Build the meshes:** `tools/cad_to_model.py parts.npz <spec> uav_simulator/urdf/<drone>` simplifies each part on its own, so small parts keep their place and size. It writes one STL per group plus `model.json`.
5. **Make the URDF:** `uav_simulator/scripts/make_cad_urdf.py <drone> quadcopter_lidar.urdf quadcopter_<drone>_cad.urdf`.
   - The Gazebo plugins come from the template, and the CAD meshes replace the visuals.
   - The LiDAR sits on a joint at the hinge, angled by `lidar_pitch_deg`.
   - `start_world.launch` publishes the LiDAR transforms from this URDF for any `*_cad.urdf` drone.

The drone then appears in the Drone gallery, drawn from its meshes. The LiDAR tilt choice (0°, 10°, 23.5°) turns the hinge in the preview and is passed to the simulator.

## Files
- `server.py`: process manager and API (Python 3.8 standard library plus numpy).
- `assets.py`: reads worlds and URDFs for the galleries.
- `terminals.py`: the debug terminals.
- `web/`: the page (three.js, roslib and xterm.js bundled in `web/vendor`, so it works without internet in the field).
- `config.json`: the buttons, topics and limits.

To ship it as a desktop app later, wrap `http://127.0.0.1:8765` in pywebview or Tauri with `server.py` as the sidecar process; nothing in the page depends on the browser.

The old dashboard (`autonomous_flight/gui/mission_control.html`) is unchanged.

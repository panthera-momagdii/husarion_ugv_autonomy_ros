# Tuning Notes — Outdoor / Sonoma Raceway

This document captures the insights, edits, and reasoning from making the
stock Husarion autonomy stack actually work on an outdoor world (Sonoma
Raceway) on a high-load workstation. Read it alongside the diffs — values
here may drift as the configs evolve.

---

## 1. Controller-spawner race in `gz_ros_control` (BOOT)

### Symptom
Nav2 lifecycle aborts on every boot. Logs spam:
```
panther.local_costmap: Invalid frame ID "panther/odom" — frame does not exist
panther.lifecycle_manager_navigation: Failed to bring up all requested nodes
panther.map_saver: Failed to spin map subscription          (every 15 s)
```

### Root cause
The `gz_ros_control` controller-manager spawns three controllers
(`joint_state_broadcaster`, `drive_controller`, `imu_broadcaster`). The
spawner's switch-controller service has a **5 s timeout**. On a busy host
(load avg ~10) the activation takes >5 s; the spawner errors out and the
controllers are left **loaded but inactive**.

With `drive_controller` inactive → no `/panther/odometry/wheels` → EKF
never publishes the `panther/odom → panther/base_link` TF → Nav2 can't
bring up `local_costmap` and times out.

### Fix (permanent)
[justfile](justfile) `start-simulation` now:
1. `docker compose up -d` (detached)
2. Waits for `controller_manager/switch_controller` service to appear
3. Calls the service explicitly to activate the three controllers
   (idempotent — succeeds whether they were already active or not)
4. Restarts the `navigation` container so Nav2 lifecycle boots on a
   healthy TF tree
5. `docker compose logs -f` to stream

### Insight
The race exists on every host; it just usually wins. On a slow box the
spawner's `--controller-manager-timeout` flag controls the *outer*
spawner timeout, not the *inner* `switch_controller` service timeout.
You can't fix this by tuning the spawner alone — you have to either
patch the launch or re-call `switch_controller` post-up. The latter is
cheap and reliable.

---

## 2. Switching the Gazebo world

### What changed
[compose.simulation.yaml](compose.simulation.yaml) gazebo service:
- Added `./worlds:/worlds` volume mount
- Added `gz_world:=${GZ_WORLD:-…default.sdf}` to the launch command

[.env](.env): documented `GZ_WORLD` and the bundled options.

### Insight
The bundled `husarion_world.sdf` (default), `husarion_office.sdf`,
`sonoma_raceway.sdf` and `empty_with_plugins.sdf` are all in the gazebo
image at `/ros2_ws/install/husarion_gz_worlds/share/husarion_gz_worlds/worlds/`.
Drop a custom `.sdf` into `./worlds/` and point `GZ_WORLD=/worlds/...`
to it.

**Important non-obvious gotcha:** swapping worlds does NOT "just work"
end-to-end — Nav2 params are scale-tuned to whatever world the defaults
were authored for. Indoor → outdoor needs the STVL retuning below.

---

## 3. GLIM live integration

### Insights
- **DDS mismatch:** the sim runs `rmw_cyclonedds_cpp`; the
  `koide3/glim_ros2:jazzy_*` image ships only `rmw_fastrtps_cpp`. They
  do not discover each other reliably. Install `ros-jazzy-rmw-cyclonedds-cpp`
  inline before launching the node.
- **CUDA tag drift:** `koide3/glim_ros2:jazzy_cuda13.1` requires a driver
  that supports CUDA 13.1. On a driver capped at CUDA 13.0, PTX JIT fails
  with `cudaErrorUnsupportedPtxVersion`. Use `:jazzy_cuda12.5` instead.
  Note that "it worked yesterday" can mean: yesterday's code path
  (e.g., `glim_rosbag` with a short bag) didn't hit the kernel that fails
  to JIT; live data does.
- **TF tree separation:** GLIM publishes its own `map → odom → panther/imu_link →
  panther/os_lidar` subtree. Nav2's stack publishes `panther/map →
  panther/odom → panther/base_link`. **Two disjoint trees** share no
  frames. Downstream consumers of GLIM output must stay inside the GLIM
  tree (e.g., `elevation_traversability` wants
  `robot_frame:=panther/os_lidar`, not `os_lidar`).
- [config/config_ros.json](../glim/config/config_ros.json) (in the GLIM
  repo) was updated so `imu_topic=/panther/imu/data` and
  `points_topic=/panther/ouster/points` match the sim.

---

## 4. STVL outdoor / variable-terrain tuning (Sonoma)

This is the bulk of the work. All edits in [config/nav2_params.yaml](config/nav2_params.yaml).

### Why the stock config fails outdoors
The defaults are tuned for the indoor `husarion_world.sdf`:
- `obstacle_range: 3.0 m` — you only mark obstacles ≤3 m away. Useless on
  a raceway.
- `voxel_size: 0.05 m` — fine indoors, but with a 128-beam Ouster
  outdoors that's ~131 k points/scan × 10 Hz = millions of voxels.
- `min_obstacle_height: 0.0` — works indoors on flat floor. Outdoors,
  the lidar sees the ground out to many meters at low Z, and **every
  ground return becomes a lethal obstacle**.

### Param-by-param diff

#### Costmap geometry
| | Old | New | Why |
|---|---|---|---|
| `local_costmap.width / height` | 8 / 8 m | **20 / 20 m** | Match the new 25 m `obstacle_range` — no point sensing 25 m into an 8 m window |
| `local_costmap.resolution` | 0.02 m | **0.10 m** | 20 m × 20 m @ 2 cm = 1 M cells. 10 cm gives 40 k — fast |
| `global_costmap.resolution` | 0.05 m | **0.10 m** | Outdoor consistency |

#### STVL (both costmaps, pointcloud + laserscan sources)
| Param | Old | New | Why |
|---|---|---|---|
| `voxel_size` | 0.05 | **0.12 m** | Internal voxel-grid downsampler. No separate filter node needed. ~6× fewer voxels |
| `voxel_min_points` | 2 | **1** | Use every point that survived the voxel filter (user asked) |
| `obstacle_range` (local) | 3.0 | **25.0** | Match real lidar reach |
| `obstacle_range` (global) | 8.0 | **40.0** | Track-scale planning |
| `min_obstacle_height` | 0.0 | **0.10** | Filter ground (lidar is ~0.4 m AGL on Panther) |
| `max_obstacle_height` | 2.0 | **2.5** | Catch barriers/cones without grabbing tall structures |
| `min_z / max_z` | 0.1 / 7.0 | **0.10 / 2.5** | Match height bounds; reject sky |
| `voxel_decay` (local) | 1.0 | **2.0** | Slightly longer persistence for sparse outdoor returns |
| `voxel_decay` (global) | 1.0 | **15.0** | Keep mapping memory across slow drives |
| `decay_acceleration` (local) | 15.0 | **10.0** | Still aggressive — avoid mark accumulation |
| `decay_acceleration` (global) | 15.0 | **5.0** | Moderate — preserves mapping but doesn't lock in |
| `unknown_threshold` | 15 | **6** | Mark sooner with sparse scans |
| `transform_tolerance` | 0.2 | **0.3 s** | Outdoor TF jitter from EKF updates |
| `expected_update_rate` | 0.0 | **1.0 Hz** | Sanity check on point flow (was disabled) |

### The big lesson: ground stamping vs variable terrain

**Failed attempt:** I first set `min_obstacle_height: -1.0` to support
slopes. Result: lidar ground returns at ~−0.4 m all marked lethal →
within a minute the entire 20×20 m local costmap was a sea of red
inflation. The 8 s `voxel_decay` I'd chosen made it worse — marks
accumulated faster than they cleared.

**Working compromise:** `min_obstacle_height: 0.10`. Ground points get
filtered, costmap stays clean. **Trade-off you accept:** moderately
steep terrain will appear as obstacles (because terrain that rises >10 cm
above the LiDAR's expected ground plane crosses the threshold). The
robot will not voluntarily drive up steep hills. For a 2D nav stack on
a wheeled robot this is arguably *correct*.

If you need to drive on slopes, the right fix is **not** more permissive
Z bounds — it's **ground segmentation upstream of STVL**. See
Suggestions below.

---

## Suggestions / next steps

### A. Ground segmentation node (recommended next)
Run a ground-segmenter (e.g. [patchworkpp_ros](https://github.com/url-kaist/patchwork-plusplus-ros),
[linefit_ground_segmentation](https://github.com/lorenwel/linefit_ground_segmentation),
or a quick RANSAC plane node) between `/panther/ouster/points_filtered`
and STVL. With non-ground points, you can safely set
`min_obstacle_height: -2.0` and **keep slope support without ground
stamping**.

### B. Add an external voxel grid filter (optional)
STVL's internal voxel filter at 0.12 m already does the downsampling.
But if you ever want a *visible* downsampled cloud to debug, run a
`pcl_ros` voxel-grid node on `/panther/ouster/points_filtered` →
`/panther/ouster/points_ds` and point STVL at the downsampled topic.

### C. Use GLIM as the SLAM (replace `slam_toolbox`)
`slam_toolbox` is 2D and pose-graph based — it struggles outdoors with
sparse features. GLIM is already running, just publishing to a separate
TF subtree. To swap:
- Edit [config/config_ros.json](../glim/config/config_ros.json):
  `odom_frame_id: panther/odom`, `map_frame_id: panther/map`.
- Launch sim with `SLAM=False` in `.env` so Nav2 doesn't run
  `slam_toolbox` (or kill the slam node).
- Disable the EKF's TF publishing or accept that wheel-odom EKF will
  fight GLIM's `odom`. Simplest: have GLIM publish only `map → odom`
  and let EKF keep `odom → base_link`.

### D. Inflation radius scaling
At 10 cm costmap resolution with `inflation_radius: 5.0 m` (global), the
gradient spans 50 cells with `cost_scaling_factor: 1.5` — very gentle.
If the Smac warning persists (`Inflation layer either not found or
inflation is not set sufficiently`), bump `cost_scaling_factor` to
~3.0–5.0 on the global costmap, or drop `inflation_radius` to ~2.5 m.
The current values are functional but not optimal for Smac2D.

### E. Make the params world-aware
The current single `nav2_params.yaml` is tuned for *outdoor*. A
`nav2_params_indoor.yaml` preset would let you keep both tunes and pick
via `.env`:
- Add `params_file:=${NAV2_PARAMS:-/config/nav2_params.yaml}` to the
  navigation service's launch in
  [compose.simulation.yaml](compose.simulation.yaml).
- Set `NAV2_PARAMS=/config/nav2_params_indoor.yaml` in `.env` for
  `husarion_office.sdf`.

### F. 3D nav stack (longer-term)
For real outdoor / terrain-aware autonomy, the 2D nav2 + STVL pipeline
is fundamentally fighting the problem. Worth evaluating:
- `nav2` + `nav2_collision_monitor` + 2.5D elevation costmap (the
  `elevation_traversability` node you already have running)
- Or a full 3D planner stack (`fields2cover`, OMPL-based 3D planners)

---

## Files touched (so far)

- [justfile](justfile) — start-simulation now self-heals the controller race
- [compose.simulation.yaml](compose.simulation.yaml) — gz_world wired through env + worlds/ mount
- [.env](.env) — documented GZ_WORLD options
- [config/nav2_params.yaml](config/nav2_params.yaml) — outdoor STVL tune + costmap geometry
- [demo-sim.md](demo-sim.md) — "Changing the Gazebo World" section added
- `worlds/` (created, empty) — drop custom SDFs here

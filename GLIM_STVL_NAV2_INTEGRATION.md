# GLIM + STVL-only Nav2 Integration

This document describes the **GLIM** (LiDAR-Inertial SLAM, [koide3/glim](https://github.com/koide3/glim)) integration with this Husarion UGV autonomy stack. It is an alternative to the stock `slam_toolbox` + `EKF` + 2D static-map setup that ships in [compose.simulation.yaml](compose.simulation.yaml). Both setups coexist — switch by running a different `just` recipe.

---

## TL;DR

```bash
# Stock setup (slam_toolbox + EKF + static layer)
just start-simulation

# GLIM setup (LiDAR-inertial SLAM + rolling-window STVL, no static map, no EKF)
just start-simulation-glim
```

The GLIM variant adds one new container (`glim`), swaps the Nav2 params file to [config/nav2_glim_stvl_params.yaml](config/nav2_glim_stvl_params.yaml), and tears down `slam_toolbox`, `amcl`, `map_server`, and `ekf_filter`. The GLIM container owns the full `<ns>/map → <ns>/odom → <ns>/base_link` TF chain.

---

## Why

The default 2D occupancy stack is built around **planar AMCL + a static yaml map** on flat indoor environments. Outdoors (e.g. the bundled `sonoma_raceway.sdf` world), 2D scan-matching wobbles, AMCL needs a pre-built static map, and the map editor toolflow is awkward.

GLIM gives us:

- **3D LiDAR-inertial odometry** that holds up on uneven terrain.
- **Global pose graph SLAM** — the `map → odom` correction is published live; no offline mapping step.
- **A drop-in replacement for EKF** — GLIM publishes `odom → base_link` from the IMU+LiDAR fusion, so we can turn EKF off.
- **No static yaml map required.** Nav2 runs in *rolling-window* mode: a `width × height` costmap travels with the robot, fed only by STVL (live 3D obstacles) + inflation. Pattern adapted from [nav2_no_map_params.yaml](https://github.com/ros-navigation/navigation2_tutorials/blob/rolling/nav2_gps_waypoint_follower_demo/config/nav2_no_map_params.yaml).

---

## Files

| Path | Purpose |
|---|---|
| [compose.simulation.glim.yaml](compose.simulation.glim.yaml) | GLIM-variant compose. Adds `glim` service; navigation uses the GLIM bringup launch; gazebo gets a `pid: host` + overlay mounts for the lidar URDF and bridge config (see DDS/throughput notes below). |
| [config/nav2_glim_stvl_params.yaml](config/nav2_glim_stvl_params.yaml) | Nav2 params: no `slam_toolbox`, no `amcl`, no `map_server`, no `static_layer`. Global costmap `rolling_window: true, width: 60, height: 60`. |
| [config/bringup_glim_stvl_launch.py](config/bringup_glim_stvl_launch.py) | Copy of `husarion_ugv_navigation/launch/bringup_launch.py` with `slam_launch` / `localization_launch` / `map_autosaver` stripped out. Adds a `topic_tools::relay` that republishes `/glim_ros/odom` as `/<ns>/odometry/glim` (keeps Nav2 namespace-pure). Uses `pointcloud_crop_be.py` (BEST_EFFORT) instead of the upstream RELIABLE C++ crop. |
| [config/pointcloud_crop_be.py](config/pointcloud_crop_be.py) | Drop-in replacement for `pointcloud_crop_box::PointcloudCropBoxNode` using BEST_EFFORT QoS on both sub and pub. Removes the only RELIABLE consumer of `/panther/ouster/points`, which was bottlenecking the gz lidar bridge under load. |
| [config/gz_ouster_os_remappings.yaml](config/gz_ouster_os_remappings.yaml) | Override of husarion_components_description's bridge config — adds `publisher_queue_size: 1, subscriber_queue_size: 1`. Mounted over the upstream file in `compose.simulation.glim.yaml`. |
| [config/ouster.urdf.xacro](config/ouster.urdf.xacro) | Override of the lidar URDF — `update_rate: 20.0 → 10.0`. The gz lidar bridge can only push ~10 Hz of PointCloud2 through DDS on this host, so generating at 20 Hz stuffs the bridge with old frames and timestamps drift behind `/clock`. |
| [../glim/config/config_ros.json](../glim/config/config_ros.json) | GLIM ROS config — frame IDs `panther/{map,odom,base_link}`, `publish_imu2lidar: false`, `enable_local_mapping: false, enable_global_mapping: false` (Nav2 only needs odom — turning off SLAM cuts CPU and ~20 publishers). |
| [../glim/config/config.json](../glim/config/config.json) | GLIM master config — switched to `config_odometry_cpu.json` (no GPU contention with gazebo's GPU rays). |
| [justfile](justfile) | New target `start-simulation-glim`. |
| [.env](.env) | `ROBOT_NAMESPACE`, `GLIM_CONFIG_DIR`. |

Everything in the stock path is untouched — `compose.simulation.yaml`, `nav2_params.yaml`, `start-simulation` all work exactly as before.

---

## Architecture

```
┌──────────────────────────────────────────────────────────────────────────┐
│ gazebo container                                                          │
│   ─ ign sim                                                               │
│   ─ gz_bridge:  /panther/ouster/points  /panther/imu/data  /panther/joint │
│   ─ ekf_filter   ←──── KILLED on startup by just start-simulation-glim    │
│   ─ robot_state_publisher → panther/base_link → urdf static frames        │
└──────────────────────────────────────────────────────────────────────────┘
            │ /panther/imu/data                /panther/ouster/points
            ▼                                  ▼
┌──────────────────────────────────────────────────────────────────────────┐
│ glim container  (koide3/glim_ros2:jazzy_cuda12.5)                         │
│   ─ glim_rosnode  --ros-args -p config_path:=/glim/config                 │
│        publishes:   TF  panther/map  → panther/odom  → panther/base_link  │
│                     /glim_ros/odom    nav_msgs/Odometry                   │
│                     /glim_ros/points  sensor_msgs/PointCloud2 (live)      │
│                     /glim_ros/map     sensor_msgs/PointCloud2 (global)    │
└──────────────────────────────────────────────────────────────────────────┘
            │ /glim_ros/odom                  TF panther/{map,odom,base_link}
            ▼                                  │
┌──────────────────────────────────────────────────────────────────────────┐
│ navigation container                                                       │
│   ─ topic_tools relay     /glim_ros/odom  →  /panther/odometry/glim       │
│   ─ pointcloud_crop_be.py /panther/ouster/points → /…/ouster/points_filtered│
│        (Python, BEST_EFFORT QoS — replaces the upstream RELIABLE C++ node) │
│   ─ pointcloud_to_laserscan → /panther/scan                                │
│   ─ nav2_container (composed)                                              │
│        local_costmap   (rolling, STVL + inflation, panther/odom frame)     │
│        global_costmap  (rolling 60×60, STVL + inflation, panther/map)      │
│        planner / controller / behaviors / waypoint / velocity_smoother     │
└──────────────────────────────────────────────────────────────────────────┘
```

### TF tree (after `just start-simulation-glim`)

```
panther/map
  └─ panther/odom            ← GLIM (map → odom hop)
       └─ panther/base_link  ← GLIM (odom → base_link hop, replaces EKF)
            ├─ panther/base_footprint    │
            ├─ panther/imu_link          │ robot_state_publisher
            ├─ panther/os_lidar          │  (URDF static)
            ├─ panther/camera_link …     │
            └─ panther/{fl,fr,rl,rr}_wheel_link  (joint_state_broadcaster)
```

Compare against the stock TF tree which has `panther/map` from `slam_toolbox`, `panther/odom → panther/base_link` from `ekf_filter`, and the wheel-odom topic `panther/odometry/wheels` feeding EKF.

---

## The lidar-bridge throughput problem (and the four fixes)

The first naive integration crashed GLIM within seconds of startup with:

```
[glim] [warning] large time difference between points and imu!!
[glim] [warning] points=472.500000 imu=605.605000 diff=133.105000
local_index: -1, data.size(): 2, total_size: 3417
terminate called after throwing an instance of 'std::out_of_range'
  what():  IndexedSlidingWindow: index out of range
```

GLIM's CT-ICP fuses IMU and lidar by timestamp. When the two streams diverge by more than the internal sliding-window depth, GLIM aborts. We saw the lidar stamps drift ~25 % per wall second behind `/clock` — within ~2 minutes the gap exceeded the window and GLIM died.

Root cause: gazebo's `gpu_lidar` plugin generates 20 Hz scans (1024 × 128 rays). The `gz_bridge` parameter_bridge can only push ~10 Hz wall-time of `PointCloud2` through DDS on this host. The difference accumulates in the bridge's internal queue. Each republished frame carries its original (now stale) sim-time stamp.

When GLIM has *no* downstream subscribers (`docker run koide3/glim` against `start-simulation`, like the user originally did), the legacy stack works because only one RELIABLE consumer (`pointcloud_crop_box`) is on the bus and there's no contention with a nav2 stack actively running. Add the full GLIM-backed nav2 stack and the bridge falls behind permanently.

Four overlapping fixes, smallest-blast-radius first:

1. **Lidar URDF: `update_rate: 20.0 → 10.0`** — [config/ouster.urdf.xacro](config/ouster.urdf.xacro), mounted over `/ros2_ws/install/husarion_components_description/share/husarion_components_description/urdf/ouster.urdf.xacro` in the gazebo container. Cuts the lidar plugin's per-second ray work in half. *This is the single highest-impact change* — without it the bridge can never catch up.
2. **gz bridge queue size 1** — [config/gz_ouster_os_remappings.yaml](config/gz_ouster_os_remappings.yaml), mounted over the upstream. `publisher_queue_size: 1, subscriber_queue_size: 1` so the bridge drops stale frames on backpressure instead of queuing them with old timestamps.
3. **BEST_EFFORT crop replacement** — [config/pointcloud_crop_be.py](config/pointcloud_crop_be.py). The upstream `pointcloud_crop_box` C++ node subscribes RELIABLE; with that gone the only RELIABLE consumer on `/panther/ouster/points` is removed and the publisher doesn't back-pressure.
4. **GLIM lightweight mode** — `enable_local_mapping: false`, `enable_global_mapping: false`, CPU odometry. GLIM publishes only what Nav2 actually needs (`/glim_ros/odom` + TF). ~20 unused publishers and the GPU-CT-ICP pipeline are removed.

After all four, lidar stamps stay within ~1 s of `/clock` indefinitely. GLIM stays in sync, costmaps update every ~1 s, Nav2 plans + drives toward goals.

If you're deploying on a host stronger than this workstation, you may be able to skip (1) and (2) and keep the 20 Hz lidar. Watch `ros2 topic echo /panther/ouster/points --once --field header.stamp` vs `ros2 topic echo /clock --once --field clock` over a few minutes — if the gap stays under ~1 s, you're fine.

---

## What changed in each file

### `config/nav2_glim_stvl_params.yaml`

Diff (conceptually) vs `nav2_params.yaml`:

- **Removed sections:** `amcl`, `map_server`, `map_autosaver`, `<namespace>/slam_toolbox`.
- **global_costmap:**
  - `plugins: [stvl_layer, inflation_layer]` (was `[static_layer, stvl_layer, inflation_layer]`).
  - Added `rolling_window: true`, `width: 60`, `height: 60` (was unspecified — the static layer dictated the size).
- **bt_navigator / controller_server odom_topic:** `odometry/glim` (was `odometry/filtered`, EKF's topic).
- **velocity_smoother.odom_topic:** `<namespace>/odometry/glim`.
- **transform_tolerance** bumped from 0.1 → 0.3 on the costmaps / behaviors / MPPI to match GLIM's publish cadence (which is per-scan, ~10 Hz, vs EKF's ~30 Hz).

Placeholders (`<namespace>/`, `<observation_topic>`, `<min_x>` …) use the same `ReplaceString` mechanism as `nav2_params.yaml` and are filled in by `bringup_glim_stvl_launch.py`.

### `config/bringup_glim_stvl_launch.py`

Identical to `husarion_ugv_navigation/launch/bringup_launch.py` except:

- Removed the `IncludeLaunchDescription(slam_launch.py)` and `IncludeLaunchDescription(localization_launch.py)` blocks.
- Removed the `map_autosaver` Node.
- Added a `topic_tools::relay` so Nav2 subscribes to the namespaced `<ns>/odometry/glim` rather than the global `/glim_ros/odom`. Keeps the params file namespace-pure.
- Added a `glim_odom_topic` launch argument.

Loaded via volume mount at `/config/bringup_glim_stvl_launch.py`, called by absolute path — no need to rebuild the navigation image.

### `../glim/config/config_ros.json`

| Field | Stock | GLIM-integrated |
|---|---|---|
| `base_frame_id` | `""` (= IMU frame) | `panther/base_link` |
| `odom_frame_id` | `odom` | `panther/odom` |
| `map_frame_id` | `map` | `panther/map` |
| `publish_imu2lidar` | `true` | `false` (URDF already provides it) |

If you change `ROBOT_NAMESPACE` in `.env`, you must also update these three fields. The mapping is mechanical (`s/panther\//<new_ns>\//g`), but JSON doesn't substitute env vars so it's a manual step. (Alternative: pre-process the JSON in the `glim` compose command with `envsubst`.)

### `compose.simulation.glim.yaml`

Mirrors `compose.simulation.yaml` plus:

- New `glim` service running `koide3/glim_ros2:jazzy_cuda12.5`, mounting `${GLIM_CONFIG_DIR}` at `/glim/config`, on `network_mode: host` with the same cyclonedds env.
- `navigation` service points at `bringup_glim_stvl_launch.py` and `nav2_glim_stvl_params.yaml`.
- All `namespace:=panther` calls now use `${ROBOT_NAMESPACE:-panther}` so the namespace is one env var away from being changeable.

### `justfile`

New target `start-simulation-glim`. It is a near-copy of `start-simulation` with one extra step: after the controllers are activated, it `pkill`s `ekf_node` inside the gazebo container so GLIM is the sole publisher of the `<ns>/odom → <ns>/base_link` edge. Then it restarts the navigation container so Nav2 lifecycle starts on a healthy TF tree (same trick as the stock target).

The reason we kill EKF post-startup rather than launching with `use_ekf:=False`: `husarion_ugv_gazebo/launch/simulation.launch.py` doesn't expose the `use_ekf` arg downward, and we don't want to fork/patch upstream launch files. `pkill` keeps everything else (the `ekf_filter` node is a regular `Node`, not a lifecycle node — no graceful shutdown path).

---

## How to run

### Simulation

```bash
cd /root/slam/husarion_ugv_autonomy_ros
just start-simulation-glim
```

Wait ~30 s for everything to spin up. The streaming logs will show:

```
gazebo       | …
glim         | [glim_ros] config loaded from /glim/config
navigation   | [bt_navigator]: Configuring
navigation   | [lifecycle_manager_navigation]: Managed nodes are active
```

In RViz (started by the gazebo container) the global frame should be `panther/map` and you should see:

- The TF tree with `panther/map → panther/odom → panther/base_link`.
- `/glim_ros/points` rendered as a coloured point cloud.
- `/glim_ros/map` rendered as a sparser global map (transient_local — appears after the first scan).
- `/panther/global_costmap/costmap` filled by STVL in a rolling 60×60 m window.
- `/panther/local_costmap/costmap` filled by STVL in a rolling 20×20 m window.

Drive the robot (e.g. via the Foxglove/WebUI joystick, or `ros2 topic pub /panther/cmd_vel`) and watch GLIM's pose graph extend. Click "2D Goal Pose" in RViz to send a Nav2 goal.

### Hardware

For hardware deployments:

1. Set `GLIM_CONFIG_DIR` in `.env` to an absolute path on the robot host that contains a real `config_sensors.json` for the deployed sensor stack (IMU noise, `T_lidar_imu`).
2. Set `ROBOT_NAMESPACE` if it differs from `panther`. **Also edit `config_ros.json`** to match (see note in [What changed](#what-changed-in-each-file)).
3. Run `just start-simulation-glim` from a host with the husarion-ugv hardware bring-up running, or wire the same `glim` service into `compose.hardware.yaml`.

---

## Debugging recipes

### "Nav2 says: invalid frame `panther/map`"

GLIM hasn't started publishing TF yet, or didn't connect to the sensor streams. Check:

```bash
docker logs glim | tail -50                                  # initialisation errors?
docker exec gazebo ros2 topic hz /panther/ouster/points      # pointcloud flowing?
docker exec gazebo ros2 topic hz /panther/imu/data            # IMU flowing?
docker exec navigation ros2 run tf2_ros tf2_echo panther/map panther/base_link
```

If `tf2_echo` shows a transform, restart navigation: `docker restart navigation`.

### "Robot jumps in RViz / TF conflict"

EKF wasn't killed and is fighting GLIM for the `odom → base_link` edge:

```bash
docker exec gazebo ros2 node list | grep ekf_filter   # should be empty
docker exec gazebo pkill -9 -f ekf_node               # kill it
docker restart navigation
```

The `start-simulation-glim` recipe does this automatically, but if you `docker compose up` directly without it you'll see the conflict.

### "Costmap is empty"

The pointcloud crop pipeline depends on `panther/os_lidar → panther/base_footprint` being available. With GLIM publishing the full TF chain this works out of the box; if STVL is empty:

```bash
docker exec navigation ros2 topic hz /panther/ouster/points_filtered
docker exec navigation ros2 topic echo /panther/local_costmap/voxel_grid --once
docker exec navigation ros2 run tf2_ros tf2_echo panther/base_footprint panther/os_lidar
```

### Tuning STVL on outdoor / dense lidar

Open [config/nav2_glim_stvl_params.yaml](config/nav2_glim_stvl_params.yaml) under `global_costmap.stvl_layer.pointcloud` and tweak `obstacle_range`, `voxel_size`, `voxel_decay`, `decay_acceleration`. See [TUNING_NOTES.md](TUNING_NOTES.md) for the outdoor (Sonoma) defaults — they apply unchanged here.

---

## What this setup does NOT do

- **No persistent map saving** — GLIM holds the global pose graph in memory; nothing is written to `./maps/`. If you want a saved map, use GLIM's own dump utility (`glim_offline`) or run the stock `start-simulation` recipe for a slam_toolbox+map_saver pass.
- **No AMCL relocalisation** — if GLIM's pose graph drifts, there's no orthogonal localisation source. For hardware deployments where relocalisation matters, run GLIM in *odometry-only* mode (set `enable_global_mapping: false` in `config_ros.json`) and run a separate map-based localiser.
- **No automatic EKF-restore.** If you `docker compose down` and then `docker compose up` (instead of using `just start-simulation-glim`), EKF will come back and you'll have a TF conflict. Always go through the `just` target.

---

## Switching back to the stock stack

Just run the other recipe:

```bash
docker compose -f compose.simulation.glim.yaml down
just start-simulation
```

Both compose files use the same container names (`gazebo`, `navigation`, `docking`), so they can't run simultaneously — `down` first.

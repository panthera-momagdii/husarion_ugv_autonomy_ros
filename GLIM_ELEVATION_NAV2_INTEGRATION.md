# GLIM + elevation_traversability + Nav2 Integration

This document describes the **elevation_traversability** integration with the GLIM SLAM stack. It is an alternative to [GLIM_STVL_NAV2_INTEGRATION.md](GLIM_STVL_NAV2_INTEGRATION.md) — instead of letting Nav2 build its costmaps from the live point cloud via STVL, this variant lets the [elevation_traversability](../elevation_traversability) package perform the terrain analysis upstream, and Nav2 just consumes the resulting `nav_msgs/OccupancyGrid` topics via two `StaticLayer` plugins.

---

## TL;DR

```bash
cd husarion_ugv_autonomy_ros
just start-simulation-glim-elevation
```

Tear down:

```bash
docker compose -f compose.simulation.glim.yaml \
               -f compose.simulation.glim.elevation.yaml \
               --profile elevation down
```

If GLIM stalls during cold-start (`docker logs glim` shows `large time difference between points and imu!!`), run `docker restart glim` and the stack will recover within ~30 s.

---

## Why this variant

STVL gives Nav2 a live 3D obstacle layer, but it is purely geometric — anything taller than `min_obstacle_height` becomes lethal, with no awareness of slope, roughness, or step height. `elevation_traversability` produces *semantic* terrain analysis:

- A rolling 2D elevation map (`grid_map_msgs/GridMap`).
- Local and global occupancy grids derived from slope / step / roughness thresholds.
- A traversability point cloud, configurable normals + smoothing radii.

Routing those products into Nav2 via `StaticLayer` lets the rest of the stack ignore the underlying lidar and reason purely in 2D occupancy — same interface as a stored map server, but updated live by the elevation pipeline.

---

## Architecture

```
┌──────────────────┐
│ gazebo (sim)     │
│ - URDF: 10 Hz    │  /panther/ouster/points
│   lidar          │  (gz_bridge, RELIABLE,
│ - gz_bridge      │   queue_size: 1)
└──────────────────┘             │
                                 ▼
                       ┌─────────────────────┐
                       │ glim                │  /glim_ros/aligned_points_corrected
                       │ - SLAM              │ ─────────────────┐
                       │ - publishes TF      │                  │
                       │   panther/map →     │                  ▼
                       │   odom → base_link  │       ┌──────────────────────┐
                       └─────────────────────┘       │ elevation_           │
                                 │                   │ traversability       │
                                 │                   │ - terrain analysis   │
                                 │                   │ - publishes:         │
                                 │                   │   /occupancy_map_    │
                                 │                   │     local (VOLATILE) │
                                 │                   │   /occupancy_map_    │
                                 │                   │     global (LATCHED) │
                                 │                   └──────────────────────┘
                                 ▼                              │
                       ┌────────────────────┐                   │
                       │ navigation (nav2)  │ <─────────────────┘
                       │ - local_costmap:   │
                       │   StaticLayer on   │
                       │   /occupancy_map_  │
                       │   local +          │
                       │   inflation        │
                       │ - global_costmap:  │
                       │   StaticLayer on   │
                       │   /occupancy_map_  │
                       │   global +         │
                       │   inflation        │
                       └────────────────────┘
```

`elevation_traversability` subscribes to `/glim_ros/aligned_points_corrected` — the loop-closure-corrected, already-aligned point cloud from GLIM. Naively, that mid-flight subscription desynced GLIM's lidar/IMU queue; the fix was to keep the *baseline* launch file (`bringup_glim_stvl_launch.py`) instead of a slimmer custom one, because the cropper + pointcloud_to_laserscan nodes it spawns are load-bearing — they keep enough subscribers on the gz_bridge's `/panther/ouster/points` to stabilise the queue. See [FIXED_ISSUES.md §3](FIXED_ISSUES.md#3-elevation_traversability-startup-desyncs-glim).

---

## Files

| Path | Purpose |
|---|---|
| [compose.simulation.glim.yaml](compose.simulation.glim.yaml) | Base compose. Unchanged from the STVL variant — same gazebo / glim / docking / navigation services. |
| [compose.simulation.glim.elevation.yaml](compose.simulation.glim.elevation.yaml) | **Compose override.** Always used as the second `-f`. Only two changes vs base: navigation runs against `nav2_glim_elevation_params.yaml`, and a profile-gated `elevation_traversability` service is added. |
| [config/nav2_glim_elevation_params.yaml](config/nav2_glim_elevation_params.yaml) | Nav2 params with STVL stripped out and replaced by two `nav2_costmap_2d::StaticLayer` plugins (one per costmap). |
| [config/bringup_glim_stvl_launch.py](config/bringup_glim_stvl_launch.py) | Reused **unchanged** from the STVL variant. The launch's `<observation_topic>` / `<stvl_layer>` placeholders are no-ops in this params file; the cropper + laserscan nodes it spawns are harmless side effects (their outputs are unused). Reusing it is load-bearing — see [FIXED_ISSUES.md](FIXED_ISSUES.md#4-removing-side-effect-subscribers-from-launch-stalls-glim). |
| [justfile](justfile) (`start-simulation-glim-elevation`) | Orchestrates the bringup. Order matters; see ["Startup ordering"](#startup-ordering) below. |
| [../elevation_traversability/config/params.yaml](../elevation_traversability/config/params.yaml) | elevation_traversability runtime config. The compose service mounts the file read-only at `/config/params.yaml`. `map_frame: panther/map` and `robot_frame: panther/base_link` align it with GLIM's TF tree. |

---

## Topic contract

| Topic | Type | QoS | Producer | Consumer |
|---|---|---|---|---|
| `/panther/ouster/points` | `sensor_msgs/PointCloud2` | RELIABLE, VOLATILE, depth 1 (bridge override) | gazebo gz_bridge | glim, pointcloud_crop_be.py, pointcloud_to_laserscan_node |
| `/panther/imu/data` | `sensor_msgs/Imu` | RELIABLE, VOLATILE, depth 1 | gazebo gz_bridge | glim |
| `/glim_ros/aligned_points_corrected` | `sensor_msgs/PointCloud2` | — | glim | elevation_traversability |
| `/glim_ros/odom` | `nav_msgs/Odometry` | — | glim | `topic_tools relay` → `/panther/odometry/glim` |
| `panther/map → panther/odom → panther/base_link` | TF | — | glim | everyone |
| `/occupancy_map_local` | `nav_msgs/OccupancyGrid` | RELIABLE, **VOLATILE**, depth 1 | elevation_traversability | nav2 local_costmap StaticLayer |
| `/occupancy_map_global` | `nav_msgs/OccupancyGrid` | RELIABLE, **TRANSIENT_LOCAL**, depth 1 | elevation_traversability | nav2 global_costmap StaticLayer |

The QoS difference between the two occupancy maps is intentional and reflected in the Nav2 params: `map_subscribe_transient_local: false` for the local layer, `true` for the global layer. Mixing those up silently drops messages because of QoS mismatch.

---

## Startup ordering

The just target replicates the proven `start-simulation-glim` sequence and only adds elevation on top. Reordering any of these steps has been observed to corrupt GLIM mid-warmup:

1. `docker compose down --remove-orphans`.
2. `docker compose pull` (skips local-only `elevation_traversability:jazzy` via `pull_policy: never`).
3. `docker compose up -d` — brings up 4 services (gazebo, glim, docking, navigation). `elevation_traversability` is profile-gated and **does not start here**.
4. Wait for `controller_manager`, load+activate controllers (retry up to 5×).
5. `pkill -9 -f ekf_node` until gone (GLIM owns `odom → base_link`).
6. Reset hardware e-stop.
7. Wait for `panther/odom → panther/base_link` TF — GLIM is live once this exists.
8. `docker restart navigation` so nav2 lifecycle activates on a clean TF tree (matches the baseline `start-simulation-glim` step).
9. Wait for `bt_navigator` to reach `active`.
10. `docker compose --profile elevation up -d elevation_traversability` — only NOW does elevation join the DDS bus.
11. Wait for `/occupancy_map_local` to start publishing.
12. Stream logs (Ctrl-C only stops the log stream; containers keep running).

The local + global costmap `StaticLayer`s subscribe lazily, so step 8/9 can complete before elevation publishes anything; the costmaps start empty and populate as soon as `/occupancy_map_local` and `/occupancy_map_global` appear.

---

## Nav2 params highlights

### Local costmap

- `global_frame: panther/odom` (rolling window), `rolling_window: true`, `width: 20`, `height: 20`, `resolution: 0.10`.
- Plugins: `[elevation_local_layer, inflation_layer]`.
- `elevation_local_layer` = `nav2_costmap_2d::StaticLayer` on `/occupancy_map_local`. `map_subscribe_transient_local: false` to match the publisher's VOLATILE durability. `trinary_costmap: false` so unobserved cells outside the lidar FOV are treated as unknown rather than lethal.
- `inflation_layer`: `cost_scaling_factor: 1.5`, `inflation_radius: 0.7` — softer falloff than the STVL tune (3.0 / 0.6) to compensate for the coarser 0.15 m elevation grid.

### Global costmap

- `global_frame: panther/map`, `rolling_window: false` (inherits width / height / origin from the source map every 2 s).
- Plugins: `[elevation_global_layer, inflation_layer]`.
- `elevation_global_layer` = `nav2_costmap_2d::StaticLayer` on `/occupancy_map_global`. `map_subscribe_transient_local: true` to match the latched publisher; new subscribers get the latest map immediately.
- `inflation_layer`: `cost_scaling_factor: 1.5`, `inflation_radius: 0.7`, `inflate_around_unknown: false`, `inflate_unknown: false` — only inflates known obstacles so the global planner can still route through unexplored cells.

### Everything else

`controller_server` (MPPI), `planner_server` (SmacPlanner2D), `behavior_server`, `waypoint_follower`, `velocity_smoother`, and the `bt_navigator` config are unchanged from `nav2_glim_stvl_params.yaml`. The `velocity_smoother.odom_topic` is still `/panther/odometry/glim` (the GLIM relay).

---

## Verifying the stack

```bash
# 1. GLIM should show odom/info lines, NOT "large time difference"
docker logs --tail 5 glim

# 2. All nav2 nodes ACTIVE
for n in local_costmap/local_costmap global_costmap/global_costmap controller_server planner_server bt_navigator; do
  docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /panther/$n"
done

# 3. Occupancy maps publishing
docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && timeout 4 ros2 topic hz /occupancy_map_local"
docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && timeout 4 ros2 topic hz /occupancy_map_global"

# 4. Costmaps publishing
docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && timeout 6 ros2 topic hz /panther/local_costmap/costmap"
docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && timeout 6 ros2 topic hz /panther/global_costmap/costmap"
```

Expected: GLIM healthy, all 5 lifecycle nodes `active`, local map ~4 Hz, global map ~0.5 Hz, local costmap ~1.25 Hz, global costmap ~0.5 Hz.

---

## Caveats

- The `elevation_traversability:jazzy` Docker image must already be built locally (`docker build` in `../elevation_traversability/docker/`). `pull_policy: never` keeps compose from trying to fetch it from a registry.
- The bundled X11 mount for `glim` (rviz_viewer plugin) means `docker restart glim` can leave the rviz_viewer in a bad state on some hosts; if a restart hangs at `load librviz_viewer.so` for >2 minutes, fall back to `docker compose ... down` + `just start-simulation-glim-elevation`.
- This variant is sim-only at present. Hardware deployment would need a parallel `compose.hardware.glim.elevation.yaml` override; the params + launch + just-target patterns transfer unchanged.

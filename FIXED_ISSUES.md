# Fixed Issues — GLIM + elevation_traversability + Nav2 Integration

This document catalogues every concrete issue hit while wiring `elevation_traversability` into the GLIM-based Nav2 stack and how each was resolved. Issues are ordered chronologically as they were encountered.

Companion docs:
- [GLIM_ELEVATION_NAV2_INTEGRATION.md](GLIM_ELEVATION_NAV2_INTEGRATION.md) — architecture and usage
- [GLIM_STVL_NAV2_INTEGRATION.md](GLIM_STVL_NAV2_INTEGRATION.md) — the baseline GLIM+STVL stack this builds on

---

## 1. `docker compose pull` failed on `elevation_traversability:jazzy`

### Symptom

```
pull access denied for elevation_traversability, repository does not exist
or may require 'docker login'
Error response from daemon: pull access denied for elevation_traversability,
repository does not exist or may require 'docker login'
```

### Root cause

`elevation_traversability:jazzy` is built locally from `../elevation_traversability/docker/Dockerfile`; it is not hosted on any registry. The default `docker compose pull` tries to fetch every service's image and fails on the missing remote.

### Fix

`pull_policy: never` on the `elevation_traversability` service. Compose skips the pull and uses whatever the local Docker daemon has cached.

**File:** [compose.simulation.glim.elevation.yaml](compose.simulation.glim.elevation.yaml) — `elevation_traversability` service block.

```yaml
elevation_traversability:
  image: elevation_traversability:jazzy
  pull_policy: never
  ...
```

---

## 2. Starting all five containers in parallel crashes GLIM

### Symptom

`docker logs glim` floods with:

```
[glim] [warning] large time difference between points and imu!!
[glim] [warning] points=3.400000 imu=125.780000 diff=122.380000
```

GLIM never publishes `panther/map → panther/odom` or `panther/odom → panther/base_link`. The local_costmap activation fails with `transform from panther/base_link to panther/odom did not become available before timeout` and the entire nav2 lifecycle aborts.

### Root cause

When all five containers come up simultaneously, GLIM's `apt-get install ros-jazzy-rmw-cyclonedds-cpp` takes ~25 s before it can subscribe to anything. During that window, the gz_bridge is producing PointCloud2 messages with no subscriber. When GLIM finally subscribes, its CT-ICP processing falls behind the IMU stream and never catches up.

### Fix

The `just` target gates `elevation_traversability` behind a compose `profiles: [elevation]` and only brings it up after:

1. `controller_manager` is live.
2. EKF has been killed (GLIM owns `odom → base_link`).
3. E-stop has been reset.
4. GLIM is publishing `panther/odom → panther/base_link`.
5. `navigation` has been restarted and `bt_navigator` is `active`.

**Files:**
- [compose.simulation.glim.elevation.yaml](compose.simulation.glim.elevation.yaml) — `profiles: [elevation]` on the `elevation_traversability` service.
- [justfile](justfile) — `start-simulation-glim-elevation` recipe brings the 4 base services up first, then `docker compose --profile elevation up -d elevation_traversability` later in the sequence.

---

## 3. `elevation_traversability` startup desyncs GLIM

### Symptom

GLIM warms up cleanly, publishes the map TF, and runs correctly — until the `elevation_traversability` container is started. Within milliseconds of elevation joining the DDS bus, GLIM logs:

```
[glim] [warning] large time gap between consecutive LiDAR frames!!
[glim] [warning] large time difference between points and imu!!
```

Confirmed by container timestamps:
- `elevation_traversability` started at `21:44:13.909`
- GLIM first warning at `21:44:13.867` (same millisecond)

### Root cause

`elevation_traversability` subscribes to `/glim_ros/aligned_points_corrected` (the GLIM-aligned, loop-closure-corrected point cloud), so it is a *downstream subscriber* of GLIM, not a sibling on the gz_bridge. A naive expectation is that adding a downstream subscriber to GLIM mid-flight is harmless — empirically it is not. With the wrong set of subscribers active on the gz_bridge's `/panther/ouster/points` at the moment elevation joins, GLIM's internal IMU/lidar queue desyncs and never recovers from a `docker restart glim` alone.

The actual stabilising factor is the *set of subscribers* on `/panther/ouster/points` — see [§4](#4-removing-side-effect-subscribers-from-launch-stalls-glim) below. Once that is right, the downstream subscription on `/glim_ros/aligned_points_corrected` no longer triggers the desync.

### Fix

No change to `raw_topic` is required — elevation stays on the package default `/glim_ros/aligned_points_corrected`. The desync is prevented upstream by [§4](#4-removing-side-effect-subscribers-from-launch-stalls-glim) (keep the baseline launch's cropper + laserscan side-effect subscribers alive).

An earlier attempt rerouted `raw_topic` to `/panther/ouster/points` (making elevation a sibling of GLIM); that worked too, but switched elevation off the corrected, aligned cloud that the package was designed for. Keeping the default topic is preferred for terrain-analysis quality.

**File:** [compose.simulation.glim.elevation.yaml](compose.simulation.glim.elevation.yaml) — `elevation_traversability.command`:

```yaml
command:
  - ros2
  - launch
  - elevation_traversability
  - elevation_traversability.launch.py
  - params_file:=/config/params.yaml
  - raw_topic:=/glim_ros/aligned_points_corrected
  - rviz:=false
```

---

## 4. Removing side-effect subscribers from launch stalls GLIM

### Symptom

After fix #3, `just start-simulation-glim` (baseline) continued to work but the new `just start-simulation-glim-elevation` still hit the GLIM time-diff stall — this time with `elevation_traversability` not yet started. The stall happens during the very first GLIM TF wait.

### Root cause

The initial new launch file (`bringup_glim_elevation_launch.py`) dropped two nodes from the baseline `bringup_glim_stvl_launch.py`:

- `pointcloud_crop_be.py` — Python cropper that subscribes to `/panther/ouster/points`.
- `pointcloud_to_laserscan_node` — subscribes to the cropped point cloud.

Both were "unused" with elevation-driven costmaps, so they were removed to simplify the launch.

That removal changed the set of DDS subscribers on `/panther/ouster/points` during GLIM's warm-up window from 3 down to 1 (just GLIM). Empirically, the missing subscribers are load-bearing: their presence keeps GLIM's lidar/IMU queue stable through CT-ICP initialization. With only GLIM subscribed, the queue desyncs.

This is the kind of fragile DDS behavior that the comment block in [config/gz_ouster_os_remappings.yaml](config/gz_ouster_os_remappings.yaml) (the bridge `publisher_queue_size: 1` override) was originally meant to mitigate — keeping the queue tiny so stale frames get dropped instead of queued. The fix only fully bites when there are multiple subscribers exercising it.

### Fix

Reuse the baseline `bringup_glim_stvl_launch.py` verbatim and just point it at `nav2_glim_elevation_params.yaml` instead of `nav2_glim_stvl_params.yaml`. The cropper + laserscan still spawn, their output topics are unused, GLIM stays healthy.

**Files:**
- [compose.simulation.glim.elevation.yaml](compose.simulation.glim.elevation.yaml) — `navigation.command` points at `bringup_glim_stvl_launch.py`.
- *(deleted)* `config/bringup_glim_elevation_launch.py` — no longer needed.

The baseline launch's `<observation_topic>`, `<observation_topic_type>`, `<scan_topic>`, `<stvl_layer>` placeholders are no-ops when applied to `nav2_glim_elevation_params.yaml` — `nav2_common.launch.ReplaceString` silently skips substitutions for placeholders that don't appear in the source file.

---

## 5. Nav2 `local_costmap` activation race when waiting for GLIM map TF

### Symptom

The just target initially had two GLIM-TF wait loops — first `odom → base_link` (180 s timeout), then `map → odom` (120 s timeout) — before restarting navigation. The second wait reliably **timed out** even though the baseline target (which only waits for `odom → base_link` then restarts navigation immediately) succeeded.

### Root cause

Blocking the just target on `map → odom` adds up to 120 s of wall-clock time during which GLIM is fighting through its initial CT-ICP frames. Under additional load from the rest of the bringup (e.g. navigation lifecycle retries) GLIM ran out of margin and stalled.

The baseline target avoids the issue by restarting navigation as soon as `odom → base_link` is up — Nav2's own lifecycle then waits internally for the rest of the TF tree to materialize, with retry behavior tuned for exactly this case.

### Fix

Drop the explicit `map → odom` wait. Restart navigation immediately after `odom → base_link` appears, identical to baseline.

**File:** [justfile](justfile) — `start-simulation-glim-elevation` recipe, between the GLIM odom-TF wait and the navigation restart.

```bash
echo "[start-simulation-glim-elevation] waiting for GLIM TF $NS/odom -> $NS/base_link..."
for i in $(seq 1 180); do
    if docker exec gazebo bash -lc "... tf2_echo $NS/odom $NS/base_link ... | grep -q 'Translation:'"; then
        break
    fi
    sleep 2
done

# (No separate map → odom wait — Nav2's lifecycle handles it internally.)

docker restart navigation
```

---

## 6. Nav2 `StaticLayer` QoS mismatch on `/occupancy_map_*`

### Symptom (would-be)

If both `map_subscribe_transient_local` flags are set the same way, one of the two occupancy maps gets silently dropped:

- `/occupancy_map_local` publisher is **VOLATILE**. A subscriber with `map_subscribe_transient_local: true` mismatches.
- `/occupancy_map_global` publisher is **TRANSIENT_LOCAL** (latched). A subscriber with `map_subscribe_transient_local: false` works at runtime but loses the latched map a lifecycle-restarting nav2 needs on activation.

### Fix

The two layers in `nav2_glim_elevation_params.yaml` are configured to match their respective publishers exactly:

| Layer | `map_topic` | `map_subscribe_transient_local` |
|---|---|---|
| `elevation_local_layer` | `/occupancy_map_local` | `false` |
| `elevation_global_layer` | `/occupancy_map_global` | `true` |

**File:** [config/nav2_glim_elevation_params.yaml](config/nav2_glim_elevation_params.yaml) — `local_costmap.elevation_local_layer` and `global_costmap.elevation_global_layer` blocks.

Publisher QoS was confirmed by `ros2 topic info --verbose /occupancy_map_local` and `/occupancy_map_global` on a live elevation_traversability container.

---

## 7. `CYCLONE_DDS_URI` from `.env` leaks into elevation_traversability container

### Symptom (latent)

The husarion `.env` file in this repo defines:

```
CYCLONE_DDS_URI=file:///config/cyclonedds.xml
```

That XML file lives in the **husarion** config directory and is mounted into the gazebo / glim / docking / navigation containers at `/config/cyclonedds.xml`. The elevation_traversability container mounts a **different** `/config` (the elevation package's own config dir, `../elevation_traversability/config/`), which does **not** contain `cyclonedds.xml`. With the env_file inherited, cyclonedds would attempt to load a non-existent config file and fall back to defaults — fragile.

### Fix

Explicitly unset `CYCLONE_DDS_URI` in the elevation service's `environment:` block. The `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` setting is kept so the DDS implementation stays consistent across containers.

**File:** [compose.simulation.glim.elevation.yaml](compose.simulation.glim.elevation.yaml) — `elevation_traversability.environment`:

```yaml
environment:
  - RMW_IMPLEMENTATION=rmw_cyclonedds_cpp
  - CYCLONE_DDS_URI=
```

---

## 8. Stray standalone `elevation_traversability` container conflicts with compose-managed one

### Symptom

Running `just start-simulation-glim-elevation` while a `docker run elevation_traversability:jazzy` from a previous manual session is still alive results in two publishers on `/occupancy_map_local` and `/occupancy_map_global`. Nav2's StaticLayer subscription receives interleaved maps from two sources; behavior is undefined.

### Fix

The just target stops any running container that uses the `elevation_traversability:jazzy` image before bringing up the compose stack:

```bash
for cid in $(docker ps -q --filter ancestor=elevation_traversability:jazzy); do
    echo "stopping stray elevation_traversability container $cid..."
    docker stop "$cid" >/dev/null || true
done
```

**File:** [justfile](justfile) — top of the `start-simulation-glim-elevation` recipe.

---

## Diagnostic playbook

If the stack fails to start cleanly, walk through these in order:

1. **`docker ps`** — are all 5 containers up?
2. **`docker logs --tail 10 glim`** — any `large time difference`? If yes, `docker restart glim`.
3. **`docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && ros2 run tf2_ros tf2_echo panther/map panther/base_link"`** — TF chain healthy?
4. **`docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /panther/bt_navigator"`** — `active`?
5. **`ros2 topic hz /occupancy_map_local`** — should be ~4 Hz.
6. **`ros2 topic hz /panther/local_costmap/costmap`** — should be ~1.25 Hz.

A failure at step 2 (GLIM) cascades to every other step. `docker restart glim` is the standard recovery — wait ~30 s for it to re-warm, the rest of the stack picks back up automatically.

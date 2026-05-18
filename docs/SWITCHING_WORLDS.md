# Switching Gazebo Worlds

The simulation stack picks its Gazebo world via the `GZ_WORLD` variable in
[`.env`](../.env). The `gazebo` service in
[`compose.simulation.glim.yaml`](../compose.simulation.glim.yaml) passes that
variable on to `simulation.launch.py` as `gz_world:=${GZ_WORLD:-...default}`.

## Three classes of worlds

### 1. Bundled worlds (shipped inside the gazebo image)

Already present at `/ros2_ws/install/husarion_gz_worlds/share/husarion_gz_worlds/worlds/`:

| File | Notes |
|------|-------|
| `husarion_world.sdf` | Default — small enclosed test arena |
| `husarion_office.sdf` | Office mock-up |
| `sonoma_raceway.sdf` | Pulls Fuel-hosted Sonoma Raceway terrain (needs internet on first run) |
| `empty_with_plugins.sdf` | Featureless plane + required plugins |

To use one of these, set the absolute container path in `.env`:
```bash
GZ_WORLD=/ros2_ws/install/husarion_gz_worlds/share/husarion_gz_worlds/worlds/husarion_office.sdf
```

### 2. Local custom worlds (under `./worlds/`)

The host's `./worlds/` directory is bind-mounted into the gazebo container at
`/worlds/`, and `/worlds` is appended to `GZ_SIM_RESOURCE_PATH` (also in
`.env`). This means:

- Any `.sdf` world file placed in `./worlds/` is reachable inside the
  container at `/worlds/<name>.sdf`.
- Any **model folder** (with `model.config` + `model.sdf`) placed in
  `./worlds/<modelname>/` is resolvable via `model://<modelname>` from
  inside a world SDF.

#### Example: baylands

The repo ships:
- `./worlds/baylands/` — the model (mesh + textures).
- `./worlds/baylands.sdf` — a world file that wraps the model into a runnable
  scene (sun, plugins, fallback ground plane, `<include>` of the model).

Activate by uncommenting in `.env`:
```bash
GZ_WORLD=/worlds/baylands.sdf
```
Then bring the stack up (or restart only gazebo if it's already running):
```bash
docker compose -f compose.simulation.glim.yaml \
               -f compose.simulation.glim.elevation.yaml \
               up -d gazebo
```

### 3. Brand-new custom world

To add an entirely new world:

1. Drop the SDF file in `./worlds/myworld.sdf`. It needs the standard
   plugins block — copy the top section of
   [`worlds/baylands.sdf`](../worlds/baylands.sdf) as a starting point.
2. If it references additional models, drop each `<modelname>/` folder
   alongside the SDF (so `model://<modelname>` resolves via
   `GZ_SIM_RESOURCE_PATH=/worlds`).
3. Set `GZ_WORLD=/worlds/myworld.sdf` in `.env`.
4. Restart `gazebo`.

## Robot spawn position

The Husarion launch hard-codes spawn at `x=0, y=-2, z=0.2` (see
[`spawn_robot.launch.py`](https://github.com/husarion/husarion_ugv_ros/blob/master/husarion_ugv_gazebo/launch/spawn_robot.launch.py)
inside the gazebo image). For worlds with terrain that isn't flat at
`(0, -2, 0.2)` (e.g. `baylands.sdf`), the robot will either fall through
the mesh, spawn inside geometry, or sit way too high.

### Override via `PANTHER_SPAWN_POSE` (recommended)

The recipe `just start-simulation-glim-elevation` reads `PANTHER_SPAWN_POSE`
from `.env` and, after the robot spawns, calls `gz service set_pose` to
teleport it. GLIM is then restarted so SLAM initialises from the new pose
instead of integrating the teleport as a giant motion.

Format: `X Y Z ROLL PITCH YAW` (meters, radians). Set in `.env`:
```bash
PANTHER_SPAWN_POSE=-5.08 10.19 1.5 0 0 0
```
Leave unset to keep the Husarion default `(0, -2, 0.2)`.

The recipe:
1. Waits for the panther entity to appear in gz.
2. Auto-detects the active gz world name (no hard-coded "baylands").
3. Converts the RPY into a quaternion (gz's set_pose takes quaternions).
4. Issues `gz service -s /world/<world>/set_pose ...`.
5. `docker restart glim` so the new pose is GLIM's initial pose.

### When you'd edit the world instead

The teleport works for most cases. The two situations where editing the
world is cleaner:

- **You want the world origin shifted permanently** (other tooling reads
  the world SDF for ground-truth comparisons): adjust the `<include>`'s
  `<pose>` in the world SDF.
- **Many objects need re-positioning together**: edit them in the world SDF
  once rather than teleporting each one at runtime.

## Verification after switching worlds

```bash
docker logs gazebo 2>&1 | grep -i 'world\|sdf' | tail
docker compose ... ps                    # gazebo should be Up, not Restarting
ros2 topic echo --once /panther/odometry/wheels   # robot is alive
```

If the robot falls indefinitely → spawn pose / terrain geometry mismatch.
If gazebo restarts in a loop → check `docker logs gazebo` for SDF-parse or
model-resolution errors (often `Unable to find file ... model://...`).

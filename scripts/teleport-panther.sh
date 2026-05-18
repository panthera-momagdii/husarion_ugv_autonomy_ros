#!/usr/bin/env bash
# teleport-panther.sh — move the panther robot to PANTHER_SPAWN_POSE
# AFTER it has spawned, then restart GLIM so SLAM reinitialises from the
# new pose. Used by `just start-simulation-glim-elevation` when a world's
# default Husarion spawn point lands the robot inside terrain.
#
# Inputs (env):
#   PANTHER_SPAWN_POSE  — "X Y Z" (meters). Orientation forced to identity.
#                         If unset/empty, the script no-ops.
#
# Idempotent: safe to call multiple times. Returns 0 on success, 0 if no
# pose to apply, non-zero only on gz/docker failure.

set -u

POSE="${PANTHER_SPAWN_POSE:-}"
if [ -z "$POSE" ]; then
    exit 0
fi

# Word-split X Y Z; default missing components to 0.
set -- $POSE
SX="${1:-0}"
SY="${2:-0}"
SZ="${3:-0}"

# Discover gz world name (retry up to 30 s for gz transport to come up).
WORLD=""
for _ in $(seq 1 30); do
    WORLD=$(docker exec gazebo bash -lc \
        "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
         gz topic -l 2>/dev/null | grep -oE '/world/[^/]+/clock' | head -1 | cut -d/ -f3")
    [ -n "$WORLD" ] && break
    sleep 1
done
if [ -z "$WORLD" ]; then
    echo "[teleport-panther] could not detect gz world name after 30 s — skipping teleport."
    exit 0
fi

# Wait for panther entity to be queryable.
for _ in $(seq 1 30); do
    if docker exec gazebo bash -lc \
        "source /opt/ros/jazzy/setup.bash 2>/dev/null; gz model -m panther --pose 2>&1 | grep -q XYZ"; then
        break
    fi
    sleep 1
done

echo "[teleport-panther] teleporting panther to ($SX, $SY, $SZ) in world '$WORLD'..."
docker exec gazebo bash -lc \
    "source /opt/ros/jazzy/setup.bash 2>/dev/null; \
     gz service -s /world/$WORLD/set_pose \
       --reqtype gz.msgs.Pose --reptype gz.msgs.Boolean --timeout 2000 \
       --req 'name: \"panther\", position: {x: $SX, y: $SY, z: $SZ}, orientation: {x: 0, y: 0, z: 0, w: 1}'" \
    || { echo "[teleport-panther] gz set_pose failed — continuing without teleport."; exit 0; }

echo "[teleport-panther] restarting glim so SLAM reinitialises from the new pose..."
docker restart glim >/dev/null

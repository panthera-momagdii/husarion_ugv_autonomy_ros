set dotenv-load # to read ROBOT_NAMESPACE from .env file

[private]
default:
    @just --list

[private]
check-husarion-webui:
    #!/bin/bash
    if ! command -v snap &> /dev/null; then
        echo "Snap is not installed. Please install Snap first and try again."
        echo "sudo apt install snapd"
        exit 1
    fi

    if ! snap list husarion-webui &> /dev/null; then
        echo "husarion-webui is not installed."
        read -p "Do you want to install husarion-webui? (y/n): " choice
        case "$choice" in
            y|Y )
                sudo snap install husarion-webui --channel=jazzy
                ;;
            n|N )
                echo "Installation aborted."
                exit 0
                ;;
            * )
                echo "Invalid input. Please respond with 'y' or 'n'."
                exit 1
                ;;
        esac
    fi

# Start navigation on User Computer inside Husarion UGV
start-hardware service="navigation docking":
    #!/bin/bash
    docker compose -f compose.hardware.yaml down
    docker compose -f compose.hardware.yaml pull
    docker compose -f compose.hardware.yaml up {{service}}


# Start Gazebo simulator with full autonomy stack
start-simulation:
    #!/bin/bash
    set -e
    NS="${ROBOT_NAMESPACE:-panther}"
    xhost +local:docker
    docker compose -f compose.simulation.yaml down
    docker compose -f compose.simulation.yaml pull
    docker compose -f compose.simulation.yaml up -d

    # Under load the gz_ros_control spawner's 5s switch-controller timeout
    # races and leaves the controllers loaded-but-inactive. Without
    # drive_controller active there's no wheel odom, EKF never publishes
    # the odom->base_link TF, and nav2 aborts bringup. Activate them
    # explicitly (idempotent), then restart nav2 so its lifecycle starts
    # on a healthy TF tree.
    echo "[start-simulation] waiting for $NS/controller_manager..."
    for i in $(seq 1 90); do
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q /$NS/controller_manager/switch_controller"; then
            break
        fi
        sleep 1
    done

    echo "[start-simulation] activating controllers..."
    docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \
        ros2 service call /$NS/controller_manager/switch_controller \
            controller_manager_msgs/srv/SwitchController \
            '{activate_controllers: [joint_state_broadcaster, drive_controller, imu_broadcaster], deactivate_controllers: [], strictness: 1, activate_asap: true, timeout: {sec: 30, nanosec: 0}}'" \
        || echo "[start-simulation] switch_controller call returned non-zero (likely already active) - continuing"

    echo "[start-simulation] restarting navigation so nav2 boots on a healthy TF tree..."
    docker restart navigation >/dev/null

    echo "[start-simulation] streaming logs (Ctrl-C to stop containers stay up; run 'docker compose -f compose.simulation.yaml down' to stop)"
    docker compose -f compose.simulation.yaml logs -f

# Start Gazebo + GLIM SLAM + STVL-only Nav2 (see GLIM_STVL_NAV2_INTEGRATION.md)
start-simulation-glim:
    #!/bin/bash
    # Note: NO `set -e`. Four containers under load means transient
    # docker-exec / ros2 service-call returns (DDS races, slow controllers,
    # daemon-killed exec sessions) are normal. Each step handles its own
    # failure mode with explicit retries.
    NS="${ROBOT_NAMESPACE:-panther}"
    xhost +local:docker
    docker compose -f compose.simulation.glim.yaml down --remove-orphans
    docker compose -f compose.simulation.glim.yaml pull
    docker compose -f compose.simulation.glim.yaml up -d

    # gz_ros_control's controller_spawner has a 10 s timeout to find the
    # controller_manager service. Under load it races and exits with
    # "Could not contact service ..." — leaving the controller_manager up
    # but with NO controllers loaded at all (not even inactive). The stock
    # workaround calls switch_controller, but that only activates
    # already-loaded controllers and fails here ("no controller with this
    # name exists"). Run the spawner ourselves now that ros2 is up; this
    # both loads + activates all three controllers in one shot.
    echo "[start-simulation-glim] waiting for $NS/controller_manager service..."
    for i in $(seq 1 120); do
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q /$NS/controller_manager/list_controllers"; then
            echo "[start-simulation-glim] controller_manager up."
            break
        fi
        sleep 1
    done

    echo "[start-simulation-glim] loading + activating controllers (retry up to 5x)..."
    for attempt in 1 2 3 4 5; do
        out=$(docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \
            ros2 run controller_manager spawner joint_state_broadcaster drive_controller imu_broadcaster \
                --controller-manager /$NS/controller_manager --activate-as-group 2>&1" 2>&1)
        if echo "$out" | grep -q "Configured and activated all the parsed controllers"; then
            echo "[start-simulation-glim] controllers loaded + activated (attempt $attempt)."
            break
        fi
        if echo "$out" | grep -q "Controller already loaded"; then
            echo "[start-simulation-glim] controllers already loaded — skipping spawner."
            break
        fi
        echo "[start-simulation-glim] spawner attempt $attempt failed, sleeping 3 s..."
        sleep 3
    done

    # GLIM publishes <ns>/odom -> <ns>/base_link directly. EKF (started by
    # husarion_ugv_gazebo's simulate_robot.launch.py) would also publish
    # that edge and cause a TF conflict. simulate_robot.launch.py doesn't
    # expose use_ekf, so kill ekf_node in-place — equivalent to launching
    # with use_ekf:=False without patching upstream. EKF starts a few
    # seconds AFTER the controllers come up, so retry until it's actually
    # gone (one kill often races with a slower-starting EKF).
    echo "[start-simulation-glim] disabling ekf_filter (GLIM owns odom -> base_link)..."
    for i in $(seq 1 10); do
        docker exec gazebo bash -c "pkill -9 -f ekf_node" 2>/dev/null || true
        sleep 2
        # ros2 node list authoritatively says whether ekf_filter is registered
        # (pgrep -f ekf_node is unreliable here — it matches its own command
        # line and other harmless processes through docker-exec wrapping).
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null | grep -q /ekf_filter\\\|/$NS/ekf_filter"; then
            echo "[start-simulation-glim]   ekf_filter still alive, killing again (try $i)..."
        else
            echo "[start-simulation-glim]   ekf_filter terminated."
            break
        fi
    done

    # The panther's hardware e-stop is asserted at boot — nav2 reports
    # "E-stop activated. Halting navigation." on every goal until reset.
    # Reset it once. (Same as pressing the physical e-stop reset button.)
    echo "[start-simulation-glim] resetting e-stop..."
    docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \
        ros2 service call /$NS/hardware/e_stop_reset std_srvs/srv/Trigger 2>&1 | tail -3" || true

    # Wait for GLIM to actually publish the $NS/odom -> $NS/base_link TF.
    # The glim container needs to apt-install ros-jazzy-rmw-cyclonedds-cpp
    # on every cold-start (~30 s) before it can publish anything. Nav2
    # lifecycle activation fails permanently if local_costmap can't look
    # up that transform within its 5 s timeout, so block here until the
    # frame exists before restarting navigation.
    echo "[start-simulation-glim] waiting for GLIM TF $NS/odom -> $NS/base_link..."
    for i in $(seq 1 180); do
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 run tf2_ros tf2_echo $NS/odom $NS/base_link 2>&1 | grep -q 'Translation:'"; then
            echo "[start-simulation-glim] GLIM TF tree up."
            break
        fi
        sleep 2
    done

    echo "[start-simulation-glim] restarting navigation so nav2 boots on a GLIM TF tree..."
    docker restart navigation >/dev/null || true

    echo "[start-simulation-glim] streaming logs (Ctrl-C to stop streaming; containers keep running — 'docker compose -f compose.simulation.glim.yaml down' to stop)"
    docker compose -f compose.simulation.glim.yaml logs -f

# Start Gazebo + GLIM SLAM + elevation_traversability + Nav2 (occupancy-grid costmaps)
start-simulation-glim-elevation:
    #!/bin/bash
    # Same orchestration as start-simulation-glim. The compose file adds
    # an elevation_traversability container that publishes
    # /occupancy_map_local + /occupancy_map_global; Nav2 drops STVL and
    # consumes those topics via two StaticLayers. See
    # config/nav2_glim_elevation_params.yaml and
    # config/bringup_glim_elevation_launch.py.
    #
    # No `set -e`: same reasoning as start-simulation-glim — transient
    # DDS / docker-exec / service-call failures are normal under load
    # and each step handles its own retries.
    NS="${ROBOT_NAMESPACE:-panther}"
    xhost +local:docker

    # Stop any standalone elevation_traversability container the user
    # may have left running from a manual `docker run` — it would
    # double-publish /occupancy_map_local + /occupancy_map_global and
    # break the StaticLayers' map subscription.
    for cid in $(docker ps -q --filter ancestor=elevation_traversability:jazzy); do
        echo "[start-simulation-glim-elevation] stopping stray elevation_traversability container $cid..."
        docker stop "$cid" >/dev/null || true
    done

    # The .elevation.yaml is a COMPOSE OVERRIDE on top of compose.simulation.glim.yaml
    # — using both -f flags keeps gazebo + glim + docking byte-identical
    # to the working `just start-simulation-glim` baseline. The override
    # only repoints `navigation` at the new bringup/params and adds the
    # `elevation_traversability` service (which is profile-gated so it
    # stays out of the default `up`).
    BASE="compose.simulation.glim.yaml"
    OVR="compose.simulation.glim.elevation.yaml"
    docker compose -f "$BASE" -f "$OVR" down --remove-orphans
    docker compose -f "$BASE" -f "$OVR" pull
    # elevation_traversability has `profiles: [elevation]` and is NOT
    # started here on purpose. Bringing up all 5 containers in parallel
    # has been observed to starve gazebo's gz_bridge during GLIM warm-up
    # ("large time difference between points and imu!!" in glim logs)
    # and the bridge never recovers. Start the 4 base services first,
    # wait until GLIM owns the map TF, then bring elevation up.
    docker compose -f "$BASE" -f "$OVR" up -d

    echo "[start-simulation-glim-elevation] waiting for $NS/controller_manager service..."
    for i in $(seq 1 120); do
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && ros2 service list 2>/dev/null | grep -q /$NS/controller_manager/list_controllers"; then
            echo "[start-simulation-glim-elevation] controller_manager up."
            break
        fi
        sleep 1
    done

    # Load + activate controllers. Spawner retries 5× because hardware
    # interfaces (gz_ros_control plugin) can need a few seconds to be
    # ready after gazebo starts — first call typically fails with
    # "Failed to configure controller". switch_controller at the end is
    # belt-and-braces: it activates the controllers in case the spawner
    # loaded them but auto-activation timed out, leaving them INACTIVE
    # (silent IMU → GLIM hangs).
    echo "[start-simulation-glim-elevation] loading + activating controllers (retry up to 5×)..."
    for attempt in 1 2 3 4 5; do
        out=$(docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \
            ros2 run controller_manager spawner joint_state_broadcaster drive_controller imu_broadcaster \
                --controller-manager /$NS/controller_manager --activate-as-group 2>&1" 2>&1)
        if echo "$out" | grep -qE "Configured and activated all the parsed controllers|Controller already loaded"; then
            echo "[start-simulation-glim-elevation] spawner OK (attempt $attempt)."
            break
        fi
        echo "[start-simulation-glim-elevation] spawner attempt $attempt failed, sleeping 3 s..."
        sleep 3
    done
    docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \
        ros2 service call /$NS/controller_manager/switch_controller \
        controller_manager_msgs/srv/SwitchController \
        '{activate_controllers: [joint_state_broadcaster, drive_controller, imu_broadcaster], deactivate_controllers: [], strictness: 1, activate_asap: true, timeout: {sec: 30, nanosec: 0}}' 2>&1 | tail -2" \
        || true

    echo "[start-simulation-glim-elevation] disabling ekf_filter (GLIM owns odom -> base_link)..."
    for i in $(seq 1 10); do
        docker exec gazebo bash -c "pkill -9 -f ekf_node" 2>/dev/null || true
        sleep 2
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && ros2 node list 2>/dev/null | grep -q /ekf_filter\\\|/$NS/ekf_filter"; then
            echo "[start-simulation-glim-elevation]   ekf_filter still alive, killing again (try $i)..."
        else
            echo "[start-simulation-glim-elevation]   ekf_filter terminated."
            break
        fi
    done

    echo "[start-simulation-glim-elevation] resetting e-stop..."
    docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && \
        ros2 service call /$NS/hardware/e_stop_reset std_srvs/srv/Trigger 2>&1 | tail -3" || true

    # Optional spawn-pose override (PANTHER_SPAWN_POSE in .env). See
    # scripts/teleport-panther.sh — no-ops when the env var is unset.
    bash scripts/teleport-panther.sh

    echo "[start-simulation-glim-elevation] waiting for GLIM TF $NS/odom -> $NS/base_link..."
    for i in $(seq 1 180); do
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 run tf2_ros tf2_echo $NS/odom $NS/base_link 2>&1 | grep -q 'Translation:'"; then
            echo "[start-simulation-glim-elevation] GLIM TF tree up."
            break
        fi
        sleep 2
    done

    # ORDERING NOTE: Match `start-simulation-glim`'s baseline sequence —
    # restart navigation IMMEDIATELY after the odom TF is up. Don't
    # wait for the map TF separately; nav2 itself waits for it as part
    # of local_costmap activation. Empirically, blocking on map TF
    # before restarting navigation gives GLIM enough time to fall
    # behind on its lidar/IMU queue under load and the rest of the
    # bringup never recovers.
    #
    # nav2's StaticLayers subscribe to /occupancy_map_* lazily — they
    # come up empty and start drawing as soon as the topics appear, so
    # the navigation restart can safely happen before elevation is up.
    #
    # elevation_traversability subscribes to /panther/ouster/points
    # directly (NOT /glim_ros/aligned_points_corrected) — this makes it
    # a sibling of GLIM on the gz_bridge rather than a downstream
    # subscriber to GLIM's output. With the old downstream wiring, GLIM
    # consistently fell into a "large time difference between points
    # and imu!!" loop the instant elevation_traversability joined; the
    # sibling wiring keeps GLIM healthy. (The override is set on the
    # `command:` of the elevation_traversability service in
    # compose.simulation.glim.elevation.yaml.)
    #
    # nav2's StaticLayers subscribe to /occupancy_map_* lazily — they
    # come up empty and start drawing as soon as the topics appear, so
    # the navigation restart can safely happen before elevation is up.
    echo "[start-simulation-glim-elevation] restarting navigation so nav2 lifecycle activates on a healthy, unloaded TF tree..."
    docker restart navigation >/dev/null || true

    echo "[start-simulation-glim-elevation] waiting for nav2 bt_navigator to reach ACTIVE..."
    for i in $(seq 1 60); do
        if docker exec navigation bash -lc "source /opt/ros/jazzy/setup.bash && ros2 lifecycle get /$NS/bt_navigator 2>/dev/null | grep -q active"; then
            echo "[start-simulation-glim-elevation] nav2 ACTIVE."
            break
        fi
        sleep 3
    done

    echo "[start-simulation-glim-elevation] starting elevation_traversability container..."
    docker compose -f compose.simulation.glim.yaml -f compose.simulation.glim.elevation.yaml \
        --profile elevation up -d elevation_traversability

    echo "[start-simulation-glim-elevation] waiting for /occupancy_map_local..."
    for i in $(seq 1 60); do
        if docker exec gazebo bash -lc "source /opt/ros/jazzy/setup.bash && timeout 2 ros2 topic echo /occupancy_map_local --once --field header 2>&1 | grep -q 'frame_id'"; then
            echo "[start-simulation-glim-elevation] /occupancy_map_local is publishing."
            break
        fi
        sleep 2
    done

    echo "[start-simulation-glim-elevation] streaming logs (Ctrl-C to stop streaming; containers keep running — 'docker compose -f compose.simulation.glim.yaml -f compose.simulation.glim.elevation.yaml down' to stop)"
    docker compose -f compose.simulation.glim.yaml -f compose.simulation.glim.elevation.yaml logs -f

# Configure and run Husarion WebUI
start-visualization: check-husarion-webui
    #!/bin/bash
    sudo cp config/foxglove-layout.json /var/snap/husarion-webui/common/foxglove-husarion-ugv-navigation.json
    sudo snap set husarion-webui webui.layout=husarion-ugv-navigation
    sudo snap set husarion-webui ros.namespace=panther
    sudo snap set husarion-webui ros.transport=rmw_cyclonedds_cpp
    sudo husarion-webui.start

    local_ip=$(hostname -I | awk '{print $1}')
    hostname=$(hostname)
    echo "Open a web browser and go to http://$local_ip:8080/ui or http://$hostname:8080/ui if your device is connected to the same Husarnet network."

# Stop Husarion WebUI
stop-visualization: check-husarion-webui
    #!/bin/bash
    sudo husarion-webui.stop

# Dock Husarion UGV to the charging dock using navigation stack
dock DOCK_NAME:
    #!/bin/bash
    docker compose -f compose.simulation.yaml exec docking bash -c \
     "source install/setup.bash && ros2 action send_goal /panther/dock_robot nav2_msgs/action/DockRobot \" {  dock_type: charging_dock, navigate_to_staging_pose: true, dock_id: {{DOCK_NAME}} }\""

# Dock Husarion UGV to the charging dock without using navigation stack
dock-direct DOCK_NAME:
    #!/bin/bash
    docker compose -f compose.simulation.yaml exec docking bash -c \
     "source install/setup.bash && ros2 action send_goal /panther/dock_robot nav2_msgs/action/DockRobot \" {  dock_type: charging_dock, navigate_to_staging_pose: false, dock_id: {{DOCK_NAME}} }\""


# Undock Husarion UGV from the charging dock
undock:
    #!/bin/bash
    docker compose -f compose.simulation.yaml exec docking bash -c \
     "source install/setup.bash && ros2 action send_goal /panther/undock_robot nav2_msgs/action/UndockRobot \" {  dock_type: charging_dock }\""

setup-os:
    bash setup_os.sh

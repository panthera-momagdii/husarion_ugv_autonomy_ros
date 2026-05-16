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

# Copyright 2024 Husarion sp. z o.o.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# bringup_glim_stvl_launch.py
#
# Variant of husarion_ugv_navigation/launch/bringup_launch.py for the
# GLIM + STVL-only Nav2 mode. Differences vs upstream:
#   * No slam_launch.py inclusion        (GLIM owns map -> odom)
#   * No localization_launch.py          (no AMCL, no map_server)
#   * No map_autosaver                   (no persistent occupancy grid)
#   * Adds an Odometry relay so Nav2's `odometry/glim` topic name is
#     remapped from GLIM's `/glim_ros/odom` publisher (kept here rather
#     than in the params file so namespacing stays consistent).
#
# Everything else — param substitution, pointcloud crop, scan
# generation, nav2 container, navigation_launch.py — is unchanged.
#
# Usage:
#   ros2 launch /config/bringup_glim_stvl_launch.py \
#     namespace:=panther \
#     observation_topic:=/panther/ouster/points \
#     params_file:=/config/nav2_glim_stvl_params.yaml \
#     glim_odom_topic:=/glim_ros/odom \
#     robot_model:=panther use_sim_time:=True

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    EnvironmentVariable,
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.descriptions import ParameterFile
from launch_ros.substitutions import FindPackageShare
from nav2_common.launch import ReplaceString, RewrittenYaml


def generate_launch_description():
    husarion_ugv_navigation = FindPackageShare("husarion_ugv_navigation")
    launch_dir = PathJoinSubstitution([husarion_ugv_navigation, "launch"])

    autostart = LaunchConfiguration("autostart")
    log_level = LaunchConfiguration("log_level")
    namespace = LaunchConfiguration("namespace")
    observation_topic = LaunchConfiguration("observation_topic")
    observation_topic_type = LaunchConfiguration("observation_topic_type")
    params_file = LaunchConfiguration("params_file")
    robot_model = LaunchConfiguration("robot_model")
    use_composition = LaunchConfiguration("use_composition")
    use_respawn = LaunchConfiguration("use_respawn")
    use_sim_time = LaunchConfiguration("use_sim_time")
    glim_odom_topic = LaunchConfiguration("glim_odom_topic")

    declare_autostart_arg = DeclareLaunchArgument(
        "autostart",
        default_value="true",
        description="Automatically startup the nav2 stack.",
    )
    declare_log_level_arg = DeclareLaunchArgument(
        "log_level",
        default_value="info",
        description="Logging level.",
        choices=["debug", "info", "warning", "error"],
    )
    declare_namespace_arg = DeclareLaunchArgument(
        "namespace",
        default_value=EnvironmentVariable("ROBOT_NAMESPACE", default_value=""),
        description="Add namespace to all launched nodes.",
    )
    declare_observation_topic_arg = DeclareLaunchArgument(
        "observation_topic",
        default_value="",
        description="Topic name for LaserScan or PointCloud2 observation messages type.",
    )
    declare_observation_topic_type_arg = DeclareLaunchArgument(
        "observation_topic_type",
        default_value="pointcloud",
        description="Observation topic type.",
        choices=["laserscan", "pointcloud"],
    )
    declare_params_file_arg = DeclareLaunchArgument(
        "params_file",
        default_value=PathJoinSubstitution(
            [husarion_ugv_navigation, "config", "nav2_params.yaml"]
        ),
        description="Path to the parameters file to use for all nav2 related nodes",
    )
    declare_robot_model_arg = DeclareLaunchArgument(
        "robot_model",
        default_value=EnvironmentVariable(name="ROBOT_MODEL_NAME", default_value="panther"),
        description="Specify robot model",
        choices=["lynx", "panther"],
    )
    declare_use_composition_arg = DeclareLaunchArgument(
        "use_composition",
        default_value="True",
        description="Whether to use composed bringup.",
    )
    declare_use_respawn_arg = DeclareLaunchArgument(
        "use_respawn",
        default_value="False",
        description="Whether to respawn if a node crashes. Applied when composition is disabled.",
    )
    declare_use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true.",
    )
    declare_glim_odom_topic_arg = DeclareLaunchArgument(
        "glim_odom_topic",
        default_value="/glim_ros/odom",
        description="Topic published by GLIM with the odom estimate (relayed to <ns>/odometry/glim for Nav2).",
    )

    param_substitutions = {"use_sim_time": use_sim_time}

    namespace_ext = PythonExpression(["'", namespace, "' + '/' if '", namespace, "' else ''"])
    scan_topic = PythonExpression(
        [
            "'scan' if '",
            observation_topic_type,
            "' == 'pointcloud' else '",
            observation_topic,
            "'",
        ]
    )

    stvl_layer = PythonExpression(
        [
            "'stvl_pointcloud_layer' if '",
            observation_topic_type,
            "' == 'pointcloud' else 'stvl_laserscan_layer'",
        ]
    )

    bb_padding = 0.03
    robot_bounding_box = {
        "panther": {
            "min_x": -0.41 - bb_padding,
            "min_y": -0.43 - bb_padding,
            "min_z": 0.05,
            "max_x": 0.41 + bb_padding,
            "max_y": 0.43 + bb_padding,
            "max_z": 0.5,
        },
        "lynx": {
            "min_x": -0.32 - bb_padding,
            "min_y": -0.27 - bb_padding,
            "min_z": 0.05,
            "max_x": 0.32 + bb_padding,
            "max_y": 0.27 + bb_padding,
            "max_z": 0.5,
        },
    }
    observation_topic_filtered = PythonExpression(
        ["'", observation_topic, "_filtered'"],
    )

    def override_params_file(robot_model_name):
        bounding_box = robot_bounding_box[robot_model_name]
        return ReplaceString(
            source_file=params_file,
            replacements={
                "<namespace>/": namespace_ext,
                "<min_x>": str(bounding_box["min_x"]),
                "<max_x>": str(bounding_box["max_x"]),
                "<min_y>": str(bounding_box["min_y"]),
                "<max_y>": str(bounding_box["max_y"]),
                "<min_z>": str(bounding_box["min_z"]),
                "<max_z>": str(bounding_box["max_z"]),
                "<observation_topic>": observation_topic,
                "<observation_topic_type>": observation_topic_type,
                "<scan_topic>": scan_topic,
                "<stvl_layer>": stvl_layer,
            },
            condition=IfCondition(
                PythonExpression(["'", robot_model, f"' == '{robot_model_name}'"])
            ),
        )

    params_file = override_params_file("panther")
    params_file = override_params_file("lynx")

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            param_rewrites=param_substitutions,
            convert_types=True,
        ),
        allow_substs=True,
    )

    # GLIM publishes its odom on an absolute topic (e.g. /glim_ros/odom).
    # Relay it into the namespaced topic Nav2 expects (e.g. /panther/odometry/glim)
    # so Nav2 can stay namespace-pure and we don't have to hardcode any
    # global topic name in the params file.
    odom_relay = Node(
        package="topic_tools",
        executable="relay",
        name="glim_odom_relay",
        namespace=namespace,
        arguments=[glim_odom_topic, "odometry/glim"],
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    bringup_cmd_group = GroupAction(
        [
            PushRosNamespace(namespace),
            # Replacement for husarion's pointcloud_crop_box::PointcloudCropBoxNode.
            # The C++ node subscribes RELIABLE; the gz_bridge publisher is also
            # RELIABLE, so it blocks on the slow consumer and the lidar stamps
            # drift behind /clock once GLIM joins the DDS bus. This Python node
            # does the same bounding-box crop but subscribes/publishes BEST_EFFORT,
            # which removes the publisher-side back-pressure so the bridge can
            # keep up with sim time. See pointcloud_crop_be.py for details.
            ExecuteProcess(
                condition=IfCondition(
                    PythonExpression(["'", observation_topic_type, "' == 'pointcloud'"])
                ),
                cmd=[
                    "python3",
                    "/config/pointcloud_crop_be.py",
                    "--ros-args",
                    "-p",
                    ["input_topic:=", observation_topic],
                    "-p",
                    ["output_topic:=", observation_topic_filtered],
                    "-p",
                    ["target_frame:=", namespace, "/base_footprint"],
                    "-p",
                    "negative:=true",
                    "-p",
                    ["min_x:=", str(robot_bounding_box["panther"]["min_x"])],
                    "-p",
                    ["max_x:=", str(robot_bounding_box["panther"]["max_x"])],
                    "-p",
                    ["min_y:=", str(robot_bounding_box["panther"]["min_y"])],
                    "-p",
                    ["max_y:=", str(robot_bounding_box["panther"]["max_y"])],
                    "-p",
                    ["min_z:=", str(robot_bounding_box["panther"]["min_z"])],
                    "-p",
                    ["max_z:=", str(robot_bounding_box["panther"]["max_z"])],
                ],
                output="screen",
            ),
            Node(
                condition=IfCondition(
                    PythonExpression(["'", observation_topic_type, "' == 'pointcloud'"])
                ),
                package="pointcloud_to_laserscan",
                executable="pointcloud_to_laserscan_node",
                name="pointcloud_to_laserscan",
                parameters=[configured_params],
                remappings=[("cloud_in", observation_topic_filtered)],
                output="screen",
            ),
            Node(
                condition=IfCondition(use_composition),
                name="nav2_container",
                package="rclcpp_components",
                executable="component_container_isolated",
                parameters=[configured_params, {"autostart": autostart}],
                arguments=["--ros-args", "--log-level", log_level],
                output="screen",
            ),
            # NOTE: slam_launch.py and localization_launch.py intentionally
            # omitted. GLIM owns map -> odom and odom -> base_link.
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([launch_dir, "navigation_launch.py"])
                ),
                launch_arguments={
                    "namespace": namespace,
                    "use_sim_time": use_sim_time,
                    "autostart": autostart,
                    "params_file": params_file,
                    "use_composition": use_composition,
                    "use_respawn": use_respawn,
                    "container_name": "nav2_container",
                }.items(),
            ),
        ]
    )

    return LaunchDescription(
        [
            SetEnvironmentVariable("RCUTILS_LOGGING_BUFFERED_STREAM", "1"),
            declare_autostart_arg,
            declare_log_level_arg,
            declare_namespace_arg,
            declare_observation_topic_arg,
            declare_observation_topic_type_arg,
            declare_params_file_arg,
            declare_robot_model_arg,
            declare_use_composition_arg,
            declare_use_respawn_arg,
            declare_use_sim_time_arg,
            declare_glim_odom_topic_arg,
            odom_relay,
            bringup_cmd_group,
        ]
    )

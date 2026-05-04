#!/usr/bin/env python3

import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, OpaqueFunction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def make_robot_planner_yaml(base_yaml_path: str, robot_name: str) -> str:
    """
    Create a temporary robot-specific planner YAML from one shared base YAML.
    Only robot_base_frame is changed.
    """
    with open(base_yaml_path, "r") as f:
        data = yaml.safe_load(f)

    data["global_costmap"]["global_costmap"]["ros__parameters"]["robot_base_frame"] = f"{robot_name}/base_link"

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"nav2_{robot_name}_",
        suffix=".yaml",
        delete=False,
    )
    yaml.safe_dump(data, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    pkg_share = get_package_share_directory("drone_slam")
    map_merge_share = get_package_share_directory("multirobot_map_merge")

    ros2_ws = os.path.expanduser("~/ros2_ws")
    offboard_ws = os.path.expanduser("~/ws_offboard_control")

    multi_slam_launch = os.path.join(pkg_share, "launch", "multi_drone_slam.launch.py")
    map_merge_launch = os.path.join(map_merge_share, "launch", "map_merge.launch.py")
    planner_yaml = os.path.join(
        ros2_ws,
        "src",
        "PX4-ROS2-SLAM-Control",
        "drone_slam",
        "config",
        "nav2_planner_merged.yaml",
    )

    robots = [
        {
            "name": "drone_0",
            "px4_prefix": "/fmu",
            "target_system": 1,
        },
        {
            "name": "drone_1",
            "px4_prefix": "/px4_1/fmu",
            "target_system": 2,
        },
    ]

    actions = []

    # Shared multi-drone SLAM
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(multi_slam_launch)
        )
    )

    # Shared global frame
    actions.append(
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_global_map_tf",
            arguments=["0", "0", "0", "0", "0", "0", "world", "global_map"],
            output="screen",
        )
    )

    # Shared map merge
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_merge_launch),
            launch_arguments={"known_init_poses": "false"}.items(),
        )
    )

    for robot in robots:
        robot_name = robot["name"]
        px4_prefix = robot["px4_prefix"]
        target_system = str(robot["target_system"])

        frontier_goal_topic = f"/{robot_name}/frontier_goal"
        goal_pose_topic = f"/{robot_name}/goal_pose"
        px4_goal_pose_topic = f"/{robot_name}/px4_goal_pose"
        odom_topic = f"/{robot_name}/odom"
        path_topic = f"/{robot_name}/nav2_path"
        marker_topic = f"/{robot_name}/frontier_markers"
        base_frame = f"{robot_name}/base_link"

        robot_planner_yaml = make_robot_planner_yaml(planner_yaml, robot_name)

        # Offboard controller
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        f"source /opt/ros/humble/setup.bash && "
                        f"source {offboard_ws}/install/setup.bash && "
                        f"python3 {offboard_ws}/src/px4_ros_com/src/examples/offboard_py/offboard_goal_control.py "
                        f"--ros-args "
                        f"-p px4_prefix:={px4_prefix} "
                        f"-p goal_topic:={px4_goal_pose_topic} "
                        f"-p target_system:={target_system}"
                    ),
                ],
                output="screen",
            )
        )

        # Planner server
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        f"source /opt/ros/humble/setup.bash && "
                        f"source {ros2_ws}/install/setup.bash && "
                        f"ros2 run nav2_planner planner_server --ros-args "
                        f"-r __ns:=/{robot_name} "
                        f"-p use_sim_time:=true "
                        f"--params-file {robot_planner_yaml}"
                    ),
                ],
                output="screen",
            )
        )

        # Lifecycle manager
        actions.append(
            ExecuteProcess(
                cmd=[
                    "bash",
                    "-lc",
                    (
                        f"source /opt/ros/humble/setup.bash && "
                        f"source {ros2_ws}/install/setup.bash && "
                        f"ros2 run nav2_lifecycle_manager lifecycle_manager --ros-args "
                        f"-r __ns:=/{robot_name} "
                        f"-p use_sim_time:=true "
                        f"-p autostart:=true "
                        f"-p node_names:=\"['planner_server']\""
                    ),
                ],
                output="screen",
            )
        )

        # Nav2 waypoint follower
        actions.append(
            Node(
                package="drone_slam",
                executable="nav2_frontier_waypoint_follower",
                name=f"{robot_name}_nav2_frontier_waypoint_follower",
                output="screen",
                parameters=[
                    {
                        "frontier_goal_topic": frontier_goal_topic,
                        "waypoint_goal_topic": goal_pose_topic,
                        "planner_action_name": f"/{robot_name}/compute_path_to_pose",
                        "map_frame": "global_map",
                        "base_frame": base_frame,
                        "path_topic": path_topic,
                        "goal_z": -3.0,
                        "waypoint_spacing": 1.2,
                    }
                ],
            )
        )

        # Map -> PX4 bridge
        actions.append(
            Node(
                package="drone_slam",
                executable="map_to_px4_goal_bridge",
                name=f"{robot_name}_map_to_px4_goal_bridge",
                output="screen",
                parameters=[
                    {
                        "input_goal_topic": goal_pose_topic,
                        "output_goal_topic": px4_goal_pose_topic,
                        "map_frame": "global_map",
                        "odom_frame": f"{robot_name}/odom",
                        "fixed_px4_z": -3.0,
                    }
                ],
            )
        )

        # Frontier selector
        actions.append(
            Node(
                package="drone_slam",
                executable="frontier_selector",
                name=f"{robot_name}_frontier_selector",
                output="screen",
                parameters=[
                    {
                        "robot_name": robot_name,
                        "map_topic": "/merged_map",
                        "odom_topic": odom_topic,
                        "goal_topic": frontier_goal_topic,
                        "marker_topic": marker_topic,
                        "goal_z": -3.0,
                    }
                ],
            )
        )

    return actions


def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

#!/usr/bin/env python3

import copy
import os
import tempfile
import yaml

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory


def make_temp_nav2_yaml(base_yaml_path: str, robot_name: str, mode: str) -> str:
    with open(base_yaml_path, "r") as f:
        base = yaml.safe_load(f)

    planner_params = copy.deepcopy(base["planner_server"]["ros__parameters"])
    costmap_params = copy.deepcopy(base["global_costmap"]["global_costmap"]["ros__parameters"])

    if mode == "local":
        global_frame = f"{robot_name}/map"
        map_topic = f"/{robot_name}/map"
    else:
        global_frame = "global_map"
        map_topic = "/merged_map"

    costmap_params["global_frame"] = global_frame
    costmap_params["robot_base_frame"] = f"{robot_name}/base_link"
    costmap_params["static_layer"]["map_topic"] = map_topic

    cfg = {
        f"/{robot_name}/planner_server": {
            "ros__parameters": planner_params
        },
        f"/{robot_name}/global_costmap/global_costmap": {
            "ros__parameters": costmap_params
        },
    }

    tmp = tempfile.NamedTemporaryFile(
        mode="w",
        prefix=f"nav2_{robot_name}_{mode}_",
        suffix=".yaml",
        delete=False,
    )
    yaml.safe_dump(cfg, tmp, sort_keys=False)
    tmp.close()
    return tmp.name


def launch_setup(context, *args, **kwargs):
    mode = LaunchConfiguration("mode").perform(context)
    startup_delay = float(LaunchConfiguration("startup_delay_sec").perform(context))
    planner_delay = float(LaunchConfiguration("planner_delay_sec").perform(context))
    autonomy_delay = float(LaunchConfiguration("autonomy_delay_sec").perform(context))
    drone1_target_system = int(LaunchConfiguration("drone1_target_system").perform(context))
    known_init_poses = LaunchConfiguration("known_init_poses").perform(context)

    ros2_ws = os.path.expanduser("~/ros2_ws")
    offboard_ws = os.path.expanduser("~/ws_offboard_control")

    drone_slam_share = get_package_share_directory("drone_slam")
    map_merge_share = get_package_share_directory("multirobot_map_merge")

    multi_slam_launch = os.path.join(drone_slam_share, "launch", "multi_drone_slam.launch.py")
    map_merge_launch = os.path.join(map_merge_share, "launch", "map_merge.launch.py")
    base_yaml = os.path.join(
        ros2_ws,
        "src",
        "PX4-ROS2-SLAM-Control",
        "drone_slam",
        "config",
        "nav2_planner_base.yaml",
    )

    robots = [
    {
        "name": "drone_0",
        "px4_prefix": "/fmu",
        "target_system": 1,
        "selection_mode": "largest",
        "goal_rank": 0,
    },
    {
        "name": "drone_1",
        "px4_prefix": "/px4_1/fmu",
        "target_system": 2,
        "selection_mode": "score",
        "goal_rank": 1,
    },
    {
        "name": "drone_2",
        "px4_prefix": "/px4_2/fmu",
        "target_system": 3,
        "selection_mode": "nearest",
        "goal_rank": 2,
    },
]
    actions = []

    # Shared SLAM first
    slam_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(multi_slam_launch)
    )
    actions.append(slam_stack)

    # Shared TF / map merge only in merged mode
    if mode == "merged":
        world_tf = Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="world_to_global_map_tf",
            arguments=["0", "0", "0", "0", "0", "0", "world", "global_map"],
            output="screen",
        )

        map_merge = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(map_merge_launch),
            launch_arguments={"known_init_poses": known_init_poses}.items(),
        )

        actions.append(TimerAction(period=startup_delay, actions=[world_tf]))
        actions.append(TimerAction(period=startup_delay + 1.0, actions=[map_merge]))

    for idx, robot in enumerate(robots):
        robot_name = robot["name"]
        px4_prefix = robot["px4_prefix"]
        target_system = robot["target_system"]
        selection_mode = robot["selection_mode"]
        goal_rank = robot["goal_rank"]

        if mode == "local":
            map_frame = f"{robot_name}/map"
            map_topic = f"/{robot_name}/map"
        else:
            map_frame = "global_map"
            map_topic = "/merged_map"

        odom_topic = f"/{robot_name}/odom"
        base_frame = f"{robot_name}/base_link"
        frontier_goal_topic = f"/{robot_name}/frontier_goal"
        goal_pose_topic = f"/{robot_name}/goal_pose"
        px4_goal_topic = f"/{robot_name}/px4_goal_pose"
        marker_topic = f"/{robot_name}/frontier_markers"
        path_topic = f"/{robot_name}/nav2_path"
        failure_topic = f"/{robot_name}/frontier_plan_failed"

        temp_yaml = make_temp_nav2_yaml(base_yaml, robot_name, mode)

        # Stagger startup per robot a little
        robot_offset = idx * 2.0

        # Planner
        planner = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                (
                    f"source /opt/ros/humble/setup.bash && "
                    f"source {ros2_ws}/install/setup.bash && "
                    f"ros2 run nav2_planner planner_server --ros-args "
                    f"-r __ns:=/{robot_name} "
                    f"-p use_sim_time:=true "
                    f"--params-file {temp_yaml}"
                )
            ],
            output="screen",
        )

        # Lifecycle
        lifecycle = ExecuteProcess(
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
                )
            ],
            output="screen",
        )

        # Offboard
        offboard = ExecuteProcess(
            cmd=[
                "bash",
                "-lc",
                (
                    f"source /opt/ros/humble/setup.bash && "
                    f"source {offboard_ws}/install/setup.bash && "
                    f"python3 {offboard_ws}/src/px4_ros_com/src/examples/offboard_py/offboard_goal_control.py "
                    f"--ros-args "
                    f"-p px4_prefix:={px4_prefix} "
                    f"-p goal_topic:={px4_goal_topic} "
                    f"-p target_system:={target_system}"
                )
            ],
            output="screen",
        )

        # Bridge
        bridge = Node(
            package="drone_slam",
            executable="map_to_px4_goal_bridge",
            name=f"{robot_name}_map_to_px4_goal_bridge",
            output="screen",
            parameters=[
                {
                    "input_goal_topic": goal_pose_topic,
                    "output_goal_topic": px4_goal_topic,
                    "map_frame": map_frame,
                    "odom_frame": f"{robot_name}/odom",
                    "fixed_px4_z": -3.0,
                }
            ],
        )

        # Waypoint follower
        waypoint_follower = Node(
            package="drone_slam",
            executable="nav2_frontier_waypoint_follower",
            name=f"{robot_name}_nav2_frontier_waypoint_follower",
            output="screen",
            parameters=[
                {
                    "frontier_goal_topic": frontier_goal_topic,
                    "waypoint_goal_topic": goal_pose_topic,
                    "planner_action_name": f"/{robot_name}/compute_path_to_pose",
                    "map_frame": map_frame,
                    "base_frame": base_frame,
                    "path_topic": path_topic,
                    "failure_topic": failure_topic,
                    "goal_z": -3.0,
                    "waypoint_spacing": 0.20,
                    "waypoint_tolerance_xy": 0.35,
                    "startup_delay_sec": max(5.0, autonomy_delay - startup_delay),
                }
            ],
        )

        # Frontier selector
        frontier_selector = Node(
            package="drone_slam",
            executable="frontier_selector",
            name=f"{robot_name}_frontier_selector",
            output="screen",
            parameters=[
                {
                    "robot_name": robot_name,
                    "map_topic": map_topic,
                    "odom_topic": odom_topic,
                    "goal_topic": frontier_goal_topic,
                    "marker_topic": marker_topic,
                    "goal_z": -3.0,
                    "selection_mode": selection_mode,
                    "goal_rank": goal_rank,
                    "failure_topic": failure_topic,
                    "failure_fallback_mode": "nearest",
                }
            ],
        )

        # Launch timing:
        # 1) planner
        # 2) lifecycle after planner starts
        # 3) offboard much later so PX4/odom/TF have time
        # 4) bridge + follower + selector after offboard and hover time
        planner_start_time = planner_delay + robot_offset
        offboard_start_time = startup_delay + robot_offset
        autonomy_start_time = autonomy_delay + robot_offset

        actions.append(TimerAction(period=planner_start_time, actions=[planner]))

        actions.append(
            RegisterEventHandler(
                OnProcessStart(
                    target_action=planner,
                    on_start=[
                        TimerAction(period=2.0, actions=[lifecycle])
                    ],
                )
            )
        )

        actions.append(TimerAction(period=offboard_start_time, actions=[offboard]))
        actions.append(TimerAction(period=autonomy_start_time, actions=[bridge]))
        actions.append(TimerAction(period=autonomy_start_time + 1.0, actions=[waypoint_follower]))
        actions.append(TimerAction(period=autonomy_start_time + 2.0, actions=[frontier_selector]))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument(
            "mode",
            default_value="local",
            description="local or merged",
        ),
        DeclareLaunchArgument(
            "known_init_poses",
            default_value="false",
            description="Whether map_merge should use known initial robot poses",
        ),
        DeclareLaunchArgument(
            "startup_delay_sec",
            default_value="18.0",
            description="Delay before offboard starts",
        ),
        DeclareLaunchArgument(
            "planner_delay_sec",
            default_value="8.0",
            description="Delay before planners start",
        ),
        DeclareLaunchArgument(
            "autonomy_delay_sec",
            default_value="30.0",
            description="Delay before selector/follower/bridge start",
        ),
        DeclareLaunchArgument(
            "drone1_target_system",
            default_value="2",
            description="PX4 target system ID for drone_1",
        ),
        OpaqueFunction(function=launch_setup),
    ])

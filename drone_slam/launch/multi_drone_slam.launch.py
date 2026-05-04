import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import SetParameter, Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('drone_slam')
    single_launch = os.path.join(pkg_share, 'launch', 'single_drone_slam.launch.py')

    robots = [
        {
            'robot_name': 'drone_0',
            'scan_topic': '/world/obstacle/model/Lidar_drone_0/model/lidar_2d_v2/link/link/sensor/lidar_2d_v2/scan',
            'px4_odom_topic': '/fmu/out/vehicle_odometry',
        },
        {
            'robot_name': 'drone_1',
            'scan_topic': '/world/obstacle/model/Lidar_drone_1/model/lidar_2d_v2/link/link/sensor/lidar_2d_v2/scan',
            'px4_odom_topic': '/px4_1/fmu/out/vehicle_odometry',
        },
        {
            'robot_name': 'drone_2',
            'scan_topic': '/world/obstacle/model/Lidar_drone_2/model/lidar_2d_v2/link/link/sensor/lidar_2d_v2/scan',
            'px4_odom_topic': '/px4_2/fmu/out/vehicle_odometry',
        },
    ]

    actions = [
        SetParameter(name='use_sim_time', value=True),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
            output='screen'
        ),
    ]

    for robot in robots:
        actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(single_launch),
                launch_arguments={
                    'robot_name': robot['robot_name'],
                    'scan_topic': robot['scan_topic'],
                    'px4_odom_topic': robot['px4_odom_topic'],
                }.items()
            )
        )

    return LaunchDescription(actions)

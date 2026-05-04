import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    robot_name = LaunchConfiguration('robot_name')
    scan_topic = LaunchConfiguration('scan_topic')
    px4_odom_topic = LaunchConfiguration('px4_odom_topic')

    return LaunchDescription([
        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument('scan_topic'),
        DeclareLaunchArgument('px4_odom_topic'),

        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                [scan_topic, '@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan']
            ],
            remappings=[
                (scan_topic, [ '/', robot_name, '/scan' ])
            ],
            output='screen'
        ),

        Node(
            package='drone_slam',
            executable='odom_converter',
            name='odom_converter',
            parameters=[{
                'use_sim_time': True,
                'input_topic': px4_odom_topic,
                'output_topic': [ '/', robot_name, '/odom' ],
                'odom_frame': [ robot_name, '/odom' ],
                'base_frame': [ robot_name, '/base_link' ],
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            arguments=[
                '0', '0', '0.26',
                '0', '0', '0',
                [robot_name, '/base_link'],
                [robot_name, '/lidar_link']
            ],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            parameters=[{
                'use_sim_time': True,
                'mode': 'mapping',
                'map_frame': [robot_name, '/map'],
                'odom_frame': [robot_name, '/odom'],
                'base_frame': [robot_name, '/base_link'],
                'provide_odom_frame': False,
                'scan_topic': [ '/', robot_name, '/scan' ],
                'resolution': 0.05,
                'publish_period_sec': 0.05,
                'max_laser_range': 10.0,
                'transform_publish_period_sec': 0.05,
                'max_update_rate_hz': 20.0,
                'scan_queue_size': 10,
                'min_laser_range': 0.1,
            }],
            output='screen'
        ),
    ])

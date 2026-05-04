from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    robot_name = LaunchConfiguration('robot_name').perform(context)
    scan_topic = LaunchConfiguration('scan_topic').perform(context)
    px4_odom_topic = LaunchConfiguration('px4_odom_topic').perform(context)

    ros_scan_topic = f'/{robot_name}/scan'
    ros_scan_fixed_topic = f'/{robot_name}/scan_fixed'
    ros_odom_topic = f'/{robot_name}/odom'

    odom_frame = f'{robot_name}/odom'
    base_frame = f'{robot_name}/base_link'
    lidar_frame = f'{robot_name}/lidar_link'
    map_frame = f'{robot_name}/map'

    return [
        Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[
                f'{scan_topic}@sensor_msgs/msg/LaserScan@gz.msgs.LaserScan'
            ],
            remappings=[
                (scan_topic, ros_scan_topic)
            ],
            output='screen'
        ),

        Node(
            package='drone_slam',
            executable='scan_frame_rewriter',
            name='scan_frame_rewriter',
            namespace=robot_name,
            parameters=[{
                'input_topic': ros_scan_topic,
                'output_topic': ros_scan_fixed_topic,
                'output_frame': lidar_frame,
            }],
            output='screen'
        ),

        Node(
            package='drone_slam',
            executable='odom_converter',
            name='odom_converter',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'input_topic': px4_odom_topic,
                'output_topic': ros_odom_topic,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
            }],
            output='screen'
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='lidar_static_tf',
            namespace=robot_name,
            arguments=[
                '0', '0', '0.26',
                '0', '0', '0',
                base_frame,
                lidar_frame
            ],
            output='screen'
        ),

        Node(
            package='slam_toolbox',
            executable='async_slam_toolbox_node',
            name='slam_toolbox',
            namespace=robot_name,
            parameters=[{
                'use_sim_time': True,
                'mode': 'mapping',
                'map_frame': map_frame,
                'odom_frame': odom_frame,
                'base_frame': base_frame,
                'laser_frame': lidar_frame,
                'provide_odom_frame': False,
                'scan_topic': ros_scan_fixed_topic,
                'resolution': 0.05,
                'publish_period_sec': 0.05,
                'max_laser_range': 10.0,
                'transform_publish_period_sec': 0.05,
                'max_update_rate_hz': 20.0,
                'scan_queue_size': 10,
                'min_laser_range': 0.1,
            }],
            remappings=[
                ('/map', f'/{robot_name}/map'),
                ('/map_metadata', f'/{robot_name}/map_metadata'),
            ],
            output='screen'
        ),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('robot_name'),
        DeclareLaunchArgument('scan_topic'),
        DeclareLaunchArgument('px4_odom_topic'),
        OpaqueFunction(function=launch_setup),
    ])

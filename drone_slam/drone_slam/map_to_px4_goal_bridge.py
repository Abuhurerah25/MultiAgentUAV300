#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseStamped

from tf2_ros import Buffer, TransformListener, TransformException
from tf2_geometry_msgs import do_transform_pose_stamped


class MapToPx4GoalBridge(Node):
    def __init__(self):
        super().__init__('map_to_px4_goal_bridge')

        self.declare_parameter('input_goal_topic', '/drone_0/goal_pose')
        self.declare_parameter('output_goal_topic', '/drone_0/px4_goal_pose')
        self.declare_parameter('map_frame', 'drone_0/map')
        self.declare_parameter('odom_frame', 'drone_0/odom')
        self.declare_parameter('fixed_px4_z', -3.0)

        self.input_goal_topic = self.get_parameter('input_goal_topic').value
        self.output_goal_topic = self.get_parameter('output_goal_topic').value
        self.map_frame = self.get_parameter('map_frame').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.fixed_px4_z = float(self.get_parameter('fixed_px4_z').value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.goal_sub = self.create_subscription(
            PoseStamped,
            self.input_goal_topic,
            self.goal_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.output_goal_topic,
            10
        )

        self.get_logger().info(
            f'map_to_px4_goal_bridge started | '
            f'input={self.input_goal_topic} output={self.output_goal_topic}'
        )

    def goal_callback(self, msg: PoseStamped):
        try:
            transform = self.tf_buffer.lookup_transform(
                self.odom_frame,
                msg.header.frame_id,
                rclpy.time.Time()
            )
        except TransformException as ex:
            self.get_logger().warn(f'Could not transform goal into {self.odom_frame}: {ex}')
            return

        try:
            goal_in_odom = do_transform_pose_stamped(msg, transform)
        except Exception as e:
            self.get_logger().warn(f'Pose transform failed: {e}')
            return

        ros_x = float(goal_in_odom.pose.position.x)
        ros_y = float(goal_in_odom.pose.position.y)

        # Inverse of odom_converter mapping:
        # ROS x = PX4 y
        # ROS y = PX4 x
        px4_x = ros_y
        px4_y = ros_x
        px4_z = self.fixed_px4_z

        out = PoseStamped()
        out.header.stamp = self.get_clock().now().to_msg()
        out.header.frame_id = 'px4_local'
        out.pose.position.x = px4_x
        out.pose.position.y = px4_y
        out.pose.position.z = px4_z
        out.pose.orientation.w = 1.0

        self.goal_pub.publish(out)

        self.get_logger().info(
            f'Converted goal | ROS odom ({ros_x:.2f}, {ros_y:.2f}) -> '
            f'PX4 ({px4_x:.2f}, {px4_y:.2f}, {px4_z:.2f})'
        )


def main(args=None):
    rclpy.init(args=args)
    node = MapToPx4GoalBridge()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanFrameRewriter(Node):
    def __init__(self):
        super().__init__('scan_frame_rewriter')

        self.declare_parameter('input_topic', '/drone_0/scan')
        self.declare_parameter('output_topic', '/drone_0/scan_fixed')
        self.declare_parameter('output_frame', 'drone_0/lidar_link')

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame = self.get_parameter('output_frame').value

        self.sub = self.create_subscription(
            LaserScan,
            input_topic,
            self.callback,
            qos_profile_sensor_data
        )
        self.pub = self.create_publisher(
            LaserScan,
            output_topic,
            qos_profile_sensor_data
        )

        self.get_logger().info(
            f'scan_frame_rewriter started | input={input_topic} '
            f'output={output_topic} frame={self.output_frame}'
        )

    def callback(self, msg: LaserScan):
        msg.header.frame_id = self.output_frame
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanFrameRewriter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import math
import numpy as np
from scipy.spatial.transform import Rotation as R

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from px4_msgs.msg import VehicleOdometry
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomConverter(Node):
    def __init__(self):
        super().__init__('odom_converter')

        # Parameters so the same node can be launched once per drone
        self.declare_parameter('input_topic', '/fmu/out/vehicle_odometry')
        self.declare_parameter('output_topic', '/drone_0/odom')
        self.declare_parameter('odom_frame', 'drone_0/odom')
        self.declare_parameter('base_frame', 'drone_0/base_link')

        self.input_topic = self.get_parameter('input_topic').value
        self.output_topic = self.get_parameter('output_topic').value
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value

        # PX4 world frames are right-handed with Z down (NED or FRD-nav style).
        # ROS world frame is ENU: X east, Y north, Z up.
        #
        # This matrix converts a vector from a PX4 world frame with axes:
        #   X forward/north, Y right/east, Z down
        # to a ROS world frame with axes:
        #   X east/right, Y north/forward, Z up
        #
        # For PX4 POSE_FRAME_FRD / VELOCITY_FRAME_FRD this preserves handedness,
        # but note that the world heading may still be arbitrary rather than true ENU.
        self.world_to_ros = np.array([
            [0.0, 1.0, 0.0],
            [1.0, 0.0, 0.0],
            [0.0, 0.0, -1.0],
        ])

        # PX4 body frame FRD -> ROS body frame FLU
        self.body_to_ros = np.array([
            [1.0,  0.0,  0.0],
            [0.0, -1.0,  0.0],
            [0.0,  0.0, -1.0],
        ])

        self.subscription = self.create_subscription(
            VehicleOdometry,
            self.input_topic,
            self.listener_callback,
            qos_profile_sensor_data
        )

        self.publisher = self.create_publisher(Odometry, self.output_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.get_logger().info(
            f'odom_converter started | input={self.input_topic} '
            f'output={self.output_topic} odom_frame={self.odom_frame} '
            f'base_frame={self.base_frame}'
        )

    @staticmethod
    def finite_vec(vec) -> bool:
        return np.all(np.isfinite(vec))

    @staticmethod
    def diag36(values6):
        cov = np.zeros((6, 6), dtype=float)
        np.fill_diagonal(cov, values6)
        return cov.reshape(-1).tolist()

    def px4_quat_to_ros_rotation(self, q_px4: np.ndarray) -> R:
        """
        Convert PX4 quaternion to a scipy Rotation in ROS conventions.

        PX4 q is documented as Hamiltonian (w, x, y, z), passive rotation
        from body to world. We convert it into a rotation matrix and change
        basis from:
          world: PX4 NED/FRD-nav -> ROS ENU-like
          body:  PX4 FRD         -> ROS FLU
        """
        if len(q_px4) != 4 or not np.all(np.isfinite(q_px4)):
            raise ValueError('Invalid PX4 quaternion')

        # PX4 (w, x, y, z) -> scipy (x, y, z, w)
        q_scipy = np.array([q_px4[1], q_px4[2], q_px4[3], q_px4[0]], dtype=float)

        r_px4 = R.from_quat(q_scipy)
        R_px4 = r_px4.as_matrix()

        # Change basis: ROS_world <- PX4_world, ROS_body <- PX4_body
        R_ros = self.world_to_ros @ R_px4 @ self.body_to_ros
        return R.from_matrix(R_ros)

    def pose_world_to_ros(self, pos_px4: np.ndarray) -> np.ndarray:
        return self.world_to_ros @ pos_px4

    def body_frd_to_flu(self, vec_frd: np.ndarray) -> np.ndarray:
        return self.body_to_ros @ vec_frd

    def world_px4_to_ros_body(self, vec_world_px4: np.ndarray, r_ros: R) -> np.ndarray:
        """
        Convert a velocity expressed in a PX4 world frame into ROS body FLU.
        Odometry.twist should be in child_frame_id, which is base_link.
        """
        vec_world_ros = self.world_to_ros @ vec_world_px4

        # r_ros is body->world in ROS conventions, so inverse() maps world->body
        vec_body_ros = r_ros.inv().apply(vec_world_ros)
        return vec_body_ros

    def build_pose_covariance(self, msg: VehicleOdometry):
        pos_var = np.array(msg.position_variance, dtype=float)
        ori_var = np.array(msg.orientation_variance, dtype=float)

        # Fallbacks if PX4 reports invalid values
        if not self.finite_vec(pos_var):
            pos_var = np.array([0.01, 0.01, 0.01], dtype=float)
        if not self.finite_vec(ori_var):
            ori_var = np.array([0.01, 0.01, 0.01], dtype=float)

        # [x, y, z, roll, pitch, yaw]
        return self.diag36([
            float(pos_var[0]),
            float(pos_var[1]),
            float(pos_var[2]),
            float(ori_var[0]),
            float(ori_var[1]),
            float(ori_var[2]),
        ])

    def build_twist_covariance(self, msg: VehicleOdometry):
        vel_var = np.array(msg.velocity_variance, dtype=float)

        if not self.finite_vec(vel_var):
            vel_var = np.array([0.01, 0.01, 0.01], dtype=float)

        # PX4 VehicleOdometry provides velocity variance, but not angular velocity variance.
        # Use a conservative fixed value for angular-rate variance.
        ang_var = np.array([0.02, 0.02, 0.02], dtype=float)

        return self.diag36([
            float(vel_var[0]),
            float(vel_var[1]),
            float(vel_var[2]),
            float(ang_var[0]),
            float(ang_var[1]),
            float(ang_var[2]),
        ])

    def listener_callback(self, msg: VehicleOdometry):
        # Validate pose frame
        if msg.pose_frame not in (
            VehicleOdometry.POSE_FRAME_NED,
            VehicleOdometry.POSE_FRAME_FRD,
        ):
            self.get_logger().warn(
                f'Unsupported pose_frame={msg.pose_frame}; expected NED or FRD. Skipping message.'
            )
            return

        pos_px4 = np.array(msg.position, dtype=float)
        if not self.finite_vec(pos_px4):
            self.get_logger().warn('Position contains NaN/Inf. Skipping message.')
            return

        try:
            r_ros = self.px4_quat_to_ros_rotation(np.array(msg.q, dtype=float))
        except Exception as e:
            self.get_logger().warn(f'Quaternion conversion failed: {e}')
            return

        # ROS Odometry message
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        # Position: PX4 world -> ROS world
        pos_ros = self.pose_world_to_ros(pos_px4)
        odom.pose.pose.position.x = float(pos_ros[0])
        odom.pose.pose.position.y = float(pos_ros[1])
        odom.pose.pose.position.z = float(pos_ros[2])

        # Orientation: PX4 body/world conventions -> ROS FLU/ENU conventions
        q_ros = r_ros.as_quat()  # [x, y, z, w]
        if q_ros[3] < 0.0:
            q_ros = -q_ros

        odom.pose.pose.orientation.x = float(q_ros[0])
        odom.pose.pose.orientation.y = float(q_ros[1])
        odom.pose.pose.orientation.z = float(q_ros[2])
        odom.pose.pose.orientation.w = float(q_ros[3])

        # Linear velocity: always publish in child_frame_id = base_link (FLU)
        vel_px4 = np.array(msg.velocity, dtype=float)
        if self.finite_vec(vel_px4):
            if msg.velocity_frame == VehicleOdometry.VELOCITY_FRAME_BODY_FRD:
                vel_body_ros = self.body_frd_to_flu(vel_px4)

            elif msg.velocity_frame in (
                VehicleOdometry.VELOCITY_FRAME_NED,
                VehicleOdometry.VELOCITY_FRAME_FRD,
            ):
                vel_body_ros = self.world_px4_to_ros_body(vel_px4, r_ros)

            else:
                self.get_logger().warn(
                    f'Unsupported velocity_frame={msg.velocity_frame}; setting linear velocity to zero.'
                )
                vel_body_ros = np.zeros(3, dtype=float)
        else:
            vel_body_ros = np.zeros(3, dtype=float)

        odom.twist.twist.linear.x = float(vel_body_ros[0])
        odom.twist.twist.linear.y = float(vel_body_ros[1])
        odom.twist.twist.linear.z = float(vel_body_ros[2])

        # Angular velocity: PX4 says this is BODY_FRD, convert to BODY_FLU
        ang_frd = np.array(msg.angular_velocity, dtype=float)
        if self.finite_vec(ang_frd):
            ang_flu = self.body_frd_to_flu(ang_frd)
        else:
            ang_flu = np.zeros(3, dtype=float)

        odom.twist.twist.angular.x = float(ang_flu[0])
        odom.twist.twist.angular.y = float(ang_flu[1])
        odom.twist.twist.angular.z = float(ang_flu[2])

        odom.pose.covariance = self.build_pose_covariance(msg)
        odom.twist.covariance = self.build_twist_covariance(msg)

        self.publisher.publish(odom)

        # TF: odom -> base_link
        t = TransformStamped()
        t.header.stamp = odom.header.stamp
        t.header.frame_id = self.odom_frame
        t.child_frame_id = self.base_frame
        t.transform.translation.x = odom.pose.pose.position.x
        t.transform.translation.y = odom.pose.pose.position.y
        t.transform.translation.z = odom.pose.pose.position.z
        t.transform.rotation = odom.pose.pose.orientation
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = OdomConverter()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import math
from typing import List, Optional

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped, PointStamped
from nav_msgs.msg import Path
from nav2_msgs.action import ComputePathToPose

from tf2_ros import Buffer, TransformListener, TransformException


class Nav2FrontierWaypointFollower(Node):
    def __init__(self):
        super().__init__('nav2_frontier_waypoint_follower')

        # Topics / frames
        self.declare_parameter('frontier_goal_topic', '/drone_0/frontier_goal')
        self.declare_parameter('waypoint_goal_topic', '/drone_0/goal_pose')
        self.declare_parameter('planner_action_name', '/compute_path_to_pose')
        self.declare_parameter('planner_id', 'GridBased')
        self.declare_parameter('map_frame', 'drone_0/map')
        self.declare_parameter('base_frame', 'drone_0/base_link')
        self.declare_parameter('path_topic', '/drone_0/nav2_path')
        self.declare_parameter('failure_topic', '/drone_0/frontier_plan_failed')

        # Path handling
        self.declare_parameter('waypoint_spacing', 0.20)
        self.declare_parameter('waypoint_tolerance_xy', 0.35)
        self.declare_parameter('corner_angle_threshold_deg', 20.0)
        self.declare_parameter('goal_z', -3.0)

        # Replanning / publishing behavior
        self.declare_parameter('frontier_replan_distance_threshold', 0.50)
        self.declare_parameter('republish_current_waypoint_period', 1.0)
        self.declare_parameter('startup_delay_sec', 5.0)

        # Stuck detection
        self.declare_parameter('stuck_timeout_sec', 6.0)
        self.declare_parameter('stuck_progress_epsilon', 0.05)

        self.frontier_goal_topic = str(self.get_parameter('frontier_goal_topic').value)
        self.waypoint_goal_topic = str(self.get_parameter('waypoint_goal_topic').value)
        self.planner_action_name = str(self.get_parameter('planner_action_name').value)
        self.planner_id = str(self.get_parameter('planner_id').value)
        self.map_frame = str(self.get_parameter('map_frame').value)
        self.base_frame = str(self.get_parameter('base_frame').value)
        self.path_topic = str(self.get_parameter('path_topic').value)
        self.failure_topic = str(self.get_parameter('failure_topic').value)

        self.waypoint_spacing = float(self.get_parameter('waypoint_spacing').value)
        self.waypoint_tolerance_xy = float(self.get_parameter('waypoint_tolerance_xy').value)
        self.corner_angle_threshold_deg = float(
            self.get_parameter('corner_angle_threshold_deg').value
        )
        self.goal_z = float(self.get_parameter('goal_z').value)

        self.frontier_replan_distance_threshold = float(
            self.get_parameter('frontier_replan_distance_threshold').value
        )
        self.republish_current_waypoint_period = float(
            self.get_parameter('republish_current_waypoint_period').value
        )
        self.startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)

        self.stuck_timeout_sec = float(self.get_parameter('stuck_timeout_sec').value)
        self.stuck_progress_epsilon = float(self.get_parameter('stuck_progress_epsilon').value)

        # State
        self.latest_frontier_goal: Optional[PoseStamped] = None
        self.active_frontier_goal: Optional[PoseStamped] = None

        self.current_path: List[PoseStamped] = []
        self.current_waypoint_index = 0
        self.following_path = False
        self.waiting_for_plan = False

        self.current_x = 0.0
        self.current_y = 0.0
        self.current_z = 0.0
        self.have_pose = False

        self.last_waypoint_publish_time = self.get_clock().now()
        self.node_start_time = self.get_clock().now()
        self.last_tf_warn_time = self.get_clock().now()

        # Stuck detection state
        self.last_progress_time = self.get_clock().now()
        self.best_dist_for_current_waypoint = float('inf')

        # ROS interfaces
        self.frontier_sub = self.create_subscription(
            PoseStamped,
            self.frontier_goal_topic,
            self.frontier_goal_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.waypoint_goal_topic,
            10
        )

        self.path_pub = self.create_publisher(
            Path,
            self.path_topic,
            10
        )

        self.failure_pub = self.create_publisher(
            PointStamped,
            self.failure_topic,
            10
        )

        self.compute_path_client = ActionClient(
            self,
            ComputePathToPose,
            self.planner_action_name
        )

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.timer = self.create_timer(0.2, self.timer_callback)

        self.get_logger().info(
            f'nav2_frontier_waypoint_follower started | '
            f'frontier_goal={self.frontier_goal_topic} '
            f'waypoint_goal={self.waypoint_goal_topic} '
            f'map_frame={self.map_frame} base_frame={self.base_frame} '
            f'goal_z={self.goal_z:.2f} failure_topic={self.failure_topic} '
            f'waypoint_spacing={self.waypoint_spacing:.2f} '
            f'waypoint_tolerance_xy={self.waypoint_tolerance_xy:.2f}'
        )

    def frontier_goal_callback(self, msg: PoseStamped):
        self.latest_frontier_goal = msg

        self.get_logger().info(
            f'Received frontier goal x={msg.pose.position.x:.2f}, '
            f'y={msg.pose.position.y:.2f}'
        )

        elapsed = (self.get_clock().now() - self.node_start_time).nanoseconds / 1e9
        if elapsed < self.startup_delay_sec:
            self.get_logger().info(
                f'Ignoring frontier goal during startup delay '
                f'({elapsed:.1f}/{self.startup_delay_sec:.1f} s)'
            )
            return

        if self.following_path and self.active_frontier_goal is not None:
            dx = msg.pose.position.x - self.active_frontier_goal.pose.position.x
            dy = msg.pose.position.y - self.active_frontier_goal.pose.position.y
            dist = math.sqrt(dx * dx + dy * dy)

            if dist < self.frontier_replan_distance_threshold:
                self.get_logger().info(
                    f'Ignoring similar frontier goal (delta={dist:.2f} m)'
                )
                return

        if not self.waiting_for_plan:
            self.request_path_to_goal(msg)

    def request_path_to_goal(self, goal_msg: PoseStamped):
        if not self.compute_path_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 ComputePathToPose action server not available')
            return

        goal = ComputePathToPose.Goal()
        goal.goal = goal_msg
        goal.planner_id = self.planner_id
        goal.use_start = False

        self.waiting_for_plan = True
        self.active_frontier_goal = goal_msg

        self.get_logger().info('Requesting Nav2 path to frontier goal...')

        future = self.compute_path_client.send_goal_async(goal)
        future.add_done_callback(self.path_goal_response_callback)

    def path_goal_response_callback(self, future):
        self.waiting_for_plan = False

        try:
            goal_handle = future.result()
        except Exception as e:
            self.get_logger().error(f'Failed to send Nav2 path goal: {e}')
            return

        if not goal_handle.accepted:
            self.get_logger().warn('Nav2 path goal was rejected')
            self.publish_failure_for_active_goal()
            return

        self.get_logger().info('Nav2 path goal accepted')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.path_result_callback)

    def path_result_callback(self, future):
        try:
            result = future.result().result
        except Exception as e:
            self.get_logger().error(f'Failed to receive Nav2 path result: {e}')
            self.publish_failure_for_active_goal()
            return

        path_msg = result.path

        if len(path_msg.poses) == 0:
            self.get_logger().warn('Nav2 returned an empty path')
            self.current_path = []
            self.following_path = False
            self.publish_failure_for_active_goal()
            return

        self.path_pub.publish(path_msg)

        sparse_waypoints = self.sparsify_path(path_msg.poses, self.waypoint_spacing)

        for wp in sparse_waypoints:
            wp.pose.position.z = self.goal_z

        self.current_path = sparse_waypoints
        self.current_waypoint_index = 0
        self.following_path = True

        self.last_progress_time = self.get_clock().now()
        self.best_dist_for_current_waypoint = float('inf')

        self.get_logger().info(
            f'Got Nav2 path with {len(path_msg.poses)} raw poses, '
            f'{len(self.current_path)} sparse waypoints'
        )

        self.publish_current_waypoint()

    def publish_failure_for_active_goal(self):
        if self.active_frontier_goal is None:
            return

        fail_msg = PointStamped()
        fail_msg.header.stamp = self.get_clock().now().to_msg()
        fail_msg.header.frame_id = self.active_frontier_goal.header.frame_id
        fail_msg.point.x = self.active_frontier_goal.pose.position.x
        fail_msg.point.y = self.active_frontier_goal.pose.position.y
        fail_msg.point.z = self.active_frontier_goal.pose.position.z
        self.failure_pub.publish(fail_msg)

        self.get_logger().warn(
            f'Published failed frontier goal | '
            f'x={fail_msg.point.x:.2f}, y={fail_msg.point.y:.2f}'
        )

    def sparsify_path(self, poses: List[PoseStamped], spacing: float) -> List[PoseStamped]:
        """
        Reduce path density while preserving important corner points.
        This helps prevent the vehicle from cutting corners near walls.
        """
        if len(poses) <= 2:
            return poses.copy()

        out = [poses[0]]
        last_kept = poses[0]

        def heading(a: PoseStamped, b: PoseStamped) -> float:
            dx = b.pose.position.x - a.pose.position.x
            dy = b.pose.position.y - a.pose.position.y
            return math.atan2(dy, dx)

        angle_thresh = math.radians(self.corner_angle_threshold_deg)

        for i in range(1, len(poses) - 1):
            prev_pose = poses[i - 1]
            pose = poses[i]
            next_pose = poses[i + 1]

            dx = pose.pose.position.x - last_kept.pose.position.x
            dy = pose.pose.position.y - last_kept.pose.position.y
            dist = math.sqrt(dx * dx + dy * dy)

            h1 = heading(prev_pose, pose)
            h2 = heading(pose, next_pose)
            dtheta = abs(h2 - h1)
            dtheta = min(dtheta, 2.0 * math.pi - dtheta)

            is_corner = dtheta >= angle_thresh

            if dist >= spacing or is_corner:
                out.append(pose)
                last_kept = pose

        out.append(poses[-1])
        return out

    def update_robot_pose_from_tf(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.map_frame,
                self.base_frame,
                rclpy.time.Time()
            )
            self.current_x = float(t.transform.translation.x)
            self.current_y = float(t.transform.translation.y)
            self.current_z = float(t.transform.translation.z)
            self.have_pose = True
        except TransformException as ex:
            self.have_pose = False

            now = self.get_clock().now()
            dt = (now - self.last_tf_warn_time).nanoseconds / 1e9
            if dt >= 2.0:
                self.get_logger().warn(
                    f'Could not transform {self.map_frame} to {self.base_frame}: {ex}'
                )
                self.last_tf_warn_time = now

    def publish_current_waypoint(self):
        if not self.following_path or self.current_waypoint_index >= len(self.current_path):
            return

        goal = self.current_path[self.current_waypoint_index]
        goal.header.stamp = self.get_clock().now().to_msg()

        self.goal_pub.publish(goal)
        self.last_waypoint_publish_time = self.get_clock().now()

        self.get_logger().info(
            f'Publishing waypoint {self.current_waypoint_index + 1}/{len(self.current_path)} | '
            f'x={goal.pose.position.x:.2f}, '
            f'y={goal.pose.position.y:.2f}, '
            f'z={goal.pose.position.z:.2f}'
        )

    def advance_waypoint(self):
        self.current_waypoint_index += 1
        self.last_progress_time = self.get_clock().now()
        self.best_dist_for_current_waypoint = float('inf')

        if self.current_waypoint_index < len(self.current_path):
            self.publish_current_waypoint()
        else:
            self.following_path = False
            self.get_logger().info('Final waypoint reached')

    def timer_callback(self):
        self.update_robot_pose_from_tf()

        if not self.following_path or not self.have_pose:
            return

        if self.current_waypoint_index >= len(self.current_path):
            self.following_path = False
            self.get_logger().info('Finished all waypoints in current path')
            return

        target = self.current_path[self.current_waypoint_index]
        tx = target.pose.position.x
        ty = target.pose.position.y

        dx = tx - self.current_x
        dy = ty - self.current_y
        dist_xy = math.sqrt(dx * dx + dy * dy)

        # Track progress toward current waypoint
        if dist_xy < self.best_dist_for_current_waypoint - self.stuck_progress_epsilon:
            self.best_dist_for_current_waypoint = dist_xy
            self.last_progress_time = self.get_clock().now()

        dt_pub = (self.get_clock().now() - self.last_waypoint_publish_time).nanoseconds / 1e9
        if dt_pub >= self.republish_current_waypoint_period:
            self.publish_current_waypoint()

        # Normal success condition
        if dist_xy < self.waypoint_tolerance_xy:
            self.get_logger().info(
                f'Waypoint {self.current_waypoint_index + 1} reached '
                f'(xy={dist_xy:.3f} m, current_z={self.current_z:.3f})'
            )
            self.advance_waypoint()
            return

        # Stuck / no-progress condition
        dt_progress = (self.get_clock().now() - self.last_progress_time).nanoseconds / 1e9
        if dt_progress >= self.stuck_timeout_sec:
            self.get_logger().warn(
                f'Waypoint {self.current_waypoint_index + 1} appears stuck '
                f'(dist_xy={dist_xy:.3f} m, no progress for {dt_progress:.1f} s). '
                f'Skipping to next waypoint.'
            )
            self.advance_waypoint()


def main(args=None):
    rclpy.init(args=args)
    node = Nav2FrontierWaypointFollower()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

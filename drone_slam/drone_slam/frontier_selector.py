#!/usr/bin/env python3

from __future__ import annotations

from dataclasses import dataclass
from enum import IntFlag
from typing import Dict, List, Optional, Tuple

import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid, Odometry
from geometry_msgs.msg import PoseStamped, PointStamped, Point
from visualization_msgs.msg import Marker


@dataclass
class Frontier:
    points: List[Tuple[float, float]]
    size: int
    x: float          # centroid x
    y: float          # centroid y
    goal_x: float     # reachable goal point near frontier
    goal_y: float     # reachable goal point near frontier
    score: float = 0.0


class PointClassification(IntFlag):
    NONE = 0
    MAP_OPEN = 1
    MAP_CLOSED = 2
    FRONTIER_OPEN = 4
    FRONTIER_CLOSED = 8


class FrontierPoint:
    def __init__(self, x: int, y: int):
        self.map_x = x
        self.map_y = y
        self.classification = PointClassification.NONE


class FrontierCache:
    def __init__(self):
        self.cache: Dict[Tuple[int, int], FrontierPoint] = {}

    def clear(self):
        self.cache.clear()

    def get_point(self, x: int, y: int) -> FrontierPoint:
        key = (x, y)
        if key not in self.cache:
            self.cache[key] = FrontierPoint(x, y)
        return self.cache[key]


class OccupancyGrid2d:
    FREE = 0
    UNKNOWN = -1

    def __init__(self, msg: OccupancyGrid):
        self.msg = msg
        self.width = int(msg.info.width)
        self.height = int(msg.info.height)
        self.resolution = float(msg.info.resolution)
        self.origin_x = float(msg.info.origin.position.x)
        self.origin_y = float(msg.info.origin.position.y)
        self.data = list(msg.data)

    def get_cost(self, mx: int, my: int) -> int:
        return self.data[self._index(mx, my)]

    def get_size_x(self) -> int:
        return self.width

    def get_size_y(self) -> int:
        return self.height

    def map_to_world(self, mx: int, my: int) -> Tuple[float, float]:
        wx = self.origin_x + (mx + 0.5) * self.resolution
        wy = self.origin_y + (my + 0.5) * self.resolution
        return wx, wy

    def world_to_map(self, wx: float, wy: float) -> Tuple[int, int]:
        if wx < self.origin_x or wy < self.origin_y:
            raise ValueError("World coordinates below map origin")

        mx = int((wx - self.origin_x) / self.resolution)
        my = int((wy - self.origin_y) / self.resolution)

        if mx < 0 or mx >= self.width or my < 0 or my >= self.height:
            raise ValueError("World coordinates outside map bounds")

        return mx, my

    def _index(self, mx: int, my: int) -> int:
        return my * self.width + mx


class FrontierSelector(Node):
    def __init__(self):
        super().__init__('frontier_selector')

        # Parameters
        self.declare_parameter('robot_name', 'drone_0')
        self.declare_parameter('map_topic', '/merged_map')
        self.declare_parameter('odom_topic', '/drone_0/odom')
        self.declare_parameter('goal_topic', '/drone_0/frontier_goal')
        self.declare_parameter('marker_topic', '/frontier_markers')
        self.declare_parameter('goal_z', -3.0)
        self.declare_parameter('min_frontier_size', 10)
        self.declare_parameter('occupied_threshold', 50)
        self.declare_parameter('goal_rank', 0)
        self.declare_parameter('selection_mode', 'score')  # score, nearest, largest, farthest
        self.declare_parameter('distance_weight', 0.7)
        self.declare_parameter('size_weight', 0.3)
        self.declare_parameter('publish_period_sec', 2.0)
        self.declare_parameter('publish_markers', True)
        self.declare_parameter('require_odom', True)
        self.declare_parameter('goal_search_radius_cells', 6)

        # Failure handling
        self.declare_parameter('failure_topic', '/drone_0/frontier_plan_failed')
        self.declare_parameter('failed_goal_reject_radius', 1.5)
        self.declare_parameter('failure_fallback_mode', 'nearest')
        self.declare_parameter('failure_count_before_fallback', 3)

        self.robot_name = str(self.get_parameter('robot_name').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)
        self.goal_topic = str(self.get_parameter('goal_topic').value)
        self.marker_topic = str(self.get_parameter('marker_topic').value)
        self.goal_z = float(self.get_parameter('goal_z').value)
        self.min_frontier_size = int(self.get_parameter('min_frontier_size').value)
        self.occupied_threshold = int(self.get_parameter('occupied_threshold').value)
        self.goal_rank = int(self.get_parameter('goal_rank').value)
        self.selection_mode = str(self.get_parameter('selection_mode').value)
        self.distance_weight = float(self.get_parameter('distance_weight').value)
        self.size_weight = float(self.get_parameter('size_weight').value)
        self.publish_markers = bool(self.get_parameter('publish_markers').value)
        self.require_odom = bool(self.get_parameter('require_odom').value)
        self.goal_search_radius_cells = int(self.get_parameter('goal_search_radius_cells').value)

        self.failure_topic = str(self.get_parameter('failure_topic').value)
        self.failed_goal_reject_radius = float(self.get_parameter('failed_goal_reject_radius').value)
        self.failure_fallback_mode = str(self.get_parameter('failure_fallback_mode').value)
        self.failure_count_before_fallback = int(
            self.get_parameter('failure_count_before_fallback').value
        )

        self.map_msg: Optional[OccupancyGrid] = None
        self.have_odom = False
        self.robot_x = 0.0
        self.robot_y = 0.0

        self.frontier_cache = FrontierCache()

        # Failure tracking
        self.have_failed_goal = False
        self.failed_goal_x = 0.0
        self.failed_goal_y = 0.0
        self.failure_count = 0

        self.map_sub = self.create_subscription(
            OccupancyGrid,
            self.map_topic,
            self.map_callback,
            10
        )
        self.odom_sub = self.create_subscription(
            Odometry,
            self.odom_topic,
            self.odom_callback,
            10
        )
        self.failure_sub = self.create_subscription(
            PointStamped,
            self.failure_topic,
            self.failure_callback,
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            self.goal_topic,
            10
        )
        self.marker_pub = self.create_publisher(
            Marker,
            self.marker_topic,
            10
        )

        period = float(self.get_parameter('publish_period_sec').value)
        self.timer = self.create_timer(period, self.timer_callback)

        self.get_logger().info(
            f'frontier_selector started | '
            f'robot={self.robot_name} map={self.map_topic} odom={self.odom_topic} '
            f'goal={self.goal_topic} marker={self.marker_topic} '
            f'selection_mode={self.selection_mode} failure_topic={self.failure_topic}'
        )

    def map_callback(self, msg: OccupancyGrid):
        self.map_msg = msg

    def odom_callback(self, msg: Odometry):
        self.robot_x = float(msg.pose.pose.position.x)
        self.robot_y = float(msg.pose.pose.position.y)
        self.have_odom = True

    def failure_callback(self, msg: PointStamped):
        self.failed_goal_x = float(msg.point.x)
        self.failed_goal_y = float(msg.point.y)
        self.have_failed_goal = True
        self.failure_count += 1

        self.get_logger().warn(
            f'Received failed frontier goal at '
            f'({self.failed_goal_x:.2f}, {self.failed_goal_y:.2f}), '
            f'failure_count={self.failure_count}'
        )

    def timer_callback(self):
        if self.map_msg is None:
            self.get_logger().warn(f'No map received yet on {self.map_topic}')
            return

        if self.require_odom and not self.have_odom:
            self.get_logger().warn(f'No odometry received yet on {self.odom_topic}')
            return

        costmap = OccupancyGrid2d(self.map_msg)

        try:
            frontiers = self.get_frontiers(costmap)
        except Exception as e:
            self.get_logger().error(f'Frontier detection failed: {e}')
            return

        if not frontiers:
            self.get_logger().warn('No frontiers found')
            self.publish_empty_marker()
            return

        best_goal = self.select_frontier(frontiers)

        if best_goal is None:
            self.get_logger().warn('No valid frontier selected')
            self.publish_empty_marker()
            return

        self.publish_goal(best_goal)
        if self.publish_markers:
            self.publish_markers_msg(frontiers, best_goal)

    def get_frontiers(self, costmap: OccupancyGrid2d) -> List[Frontier]:
        self.frontier_cache.clear()

        try:
            mx, my = costmap.world_to_map(self.robot_x, self.robot_y)
        except Exception:
            mx = costmap.get_size_x() // 2
            my = costmap.get_size_y() // 2

        free_mx, free_my = self.find_nearest_free(mx, my, costmap)
        start = self.frontier_cache.get_point(free_mx, free_my)
        start.classification |= PointClassification.MAP_OPEN

        map_queue: List[FrontierPoint] = [start]
        frontiers: List[Frontier] = []

        while map_queue:
            p = map_queue.pop(0)

            if p.classification & PointClassification.MAP_CLOSED:
                continue

            if self.is_frontier_point(p, costmap):
                p.classification |= PointClassification.FRONTIER_OPEN
                frontier_queue: List[FrontierPoint] = [p]
                new_frontier: List[FrontierPoint] = []

                while frontier_queue:
                    q = frontier_queue.pop(0)

                    if q.classification & (
                        PointClassification.MAP_CLOSED | PointClassification.FRONTIER_CLOSED
                    ):
                        continue

                    if self.is_frontier_point(q, costmap):
                        new_frontier.append(q)

                        for w in self.get_neighbors(q, costmap):
                            if not (
                                w.classification & (
                                    PointClassification.FRONTIER_OPEN
                                    | PointClassification.FRONTIER_CLOSED
                                    | PointClassification.MAP_CLOSED
                                )
                            ):
                                w.classification |= PointClassification.FRONTIER_OPEN
                                frontier_queue.append(w)

                    q.classification |= PointClassification.FRONTIER_CLOSED

                world_points: List[Tuple[float, float]] = []
                for pt in new_frontier:
                    pt.classification |= PointClassification.MAP_CLOSED
                    world_points.append(costmap.map_to_world(pt.map_x, pt.map_y))

                if len(new_frontier) >= self.min_frontier_size:
                    cx, cy = self.centroid(world_points)
                    gx, gy = self.choose_reachable_goal_point(world_points, costmap)

                    frontiers.append(
                        Frontier(
                            points=world_points,
                            size=len(new_frontier),
                            x=cx,
                            y=cy,
                            goal_x=gx,
                            goal_y=gy,
                            score=0.0,
                        )
                    )

            for v in self.get_neighbors(p, costmap):
                if not (
                    v.classification & (PointClassification.MAP_OPEN | PointClassification.MAP_CLOSED)
                ):
                    if any(
                        costmap.get_cost(n.map_x, n.map_y) == OccupancyGrid2d.FREE
                        for n in self.get_neighbors(v, costmap)
                    ):
                        v.classification |= PointClassification.MAP_OPEN
                        map_queue.append(v)

            p.classification |= PointClassification.MAP_CLOSED

        self.get_logger().info(f'Found {len(frontiers)} frontier regions')
        return frontiers

    def find_nearest_free(self, mx: int, my: int, costmap: OccupancyGrid2d) -> Tuple[int, int]:
        local_cache = FrontierCache()
        q = [local_cache.get_point(mx, my)]
        visited = {(mx, my)}

        while q:
            loc = q.pop(0)

            if (
                0 <= loc.map_x < costmap.get_size_x()
                and 0 <= loc.map_y < costmap.get_size_y()
                and costmap.get_cost(loc.map_x, loc.map_y) == OccupancyGrid2d.FREE
            ):
                return loc.map_x, loc.map_y

            for n in self.get_neighbors(loc, costmap, local_cache):
                key = (n.map_x, n.map_y)
                if key not in visited:
                    visited.add(key)
                    q.append(n)

        return mx, my

    def get_neighbors(
        self,
        point: FrontierPoint,
        costmap: OccupancyGrid2d,
        cache: Optional[FrontierCache] = None
    ) -> List[FrontierPoint]:
        if cache is None:
            cache = self.frontier_cache

        neighbors: List[FrontierPoint] = []
        for x in range(point.map_x - 1, point.map_x + 2):
            for y in range(point.map_y - 1, point.map_y + 2):
                if x == point.map_x and y == point.map_y:
                    continue
                if 0 <= x < costmap.get_size_x() and 0 <= y < costmap.get_size_y():
                    neighbors.append(cache.get_point(x, y))
        return neighbors

    def is_frontier_point(self, point: FrontierPoint, costmap: OccupancyGrid2d) -> bool:
        if costmap.get_cost(point.map_x, point.map_y) != OccupancyGrid2d.UNKNOWN:
            return False

        has_free = False
        for n in self.get_neighbors(point, costmap):
            cost = costmap.get_cost(n.map_x, n.map_y)

            if cost >= self.occupied_threshold:
                return False

            if cost == OccupancyGrid2d.FREE:
                has_free = True

        return has_free

    @staticmethod
    def centroid(points: List[Tuple[float, float]]) -> Tuple[float, float]:
        sx = sum(p[0] for p in points)
        sy = sum(p[1] for p in points)
        n = len(points)
        return sx / n, sy / n

    def choose_reachable_goal_point(
        self,
        frontier_points: List[Tuple[float, float]],
        costmap: OccupancyGrid2d
    ) -> Tuple[float, float]:
        if not frontier_points:
            return self.robot_x, self.robot_y

        cx, cy = self.centroid(frontier_points)

        if self.selection_mode == 'nearest':
            rep_x, rep_y = min(
                frontier_points,
                key=lambda p: math.hypot(p[0] - self.robot_x, p[1] - self.robot_y)
            )
        elif self.selection_mode == 'farthest':
            rep_x, rep_y = max(
                frontier_points,
                key=lambda p: math.hypot(p[0] - self.robot_x, p[1] - self.robot_y)
            )
        else:
            rep_x, rep_y = min(
                frontier_points,
                key=lambda p: math.hypot(p[0] - cx, p[1] - cy)
            )

        try:
            mx, my = costmap.world_to_map(rep_x, rep_y)
        except Exception:
            return rep_x, rep_y

        best_free: Optional[Tuple[float, float]] = None
        best_dist = float('inf')
        search_radius = max(1, self.goal_search_radius_cells)

        for dx in range(-search_radius, search_radius + 1):
            for dy in range(-search_radius, search_radius + 1):
                nx = mx + dx
                ny = my + dy

                if nx < 0 or ny < 0 or nx >= costmap.get_size_x() or ny >= costmap.get_size_y():
                    continue

                if costmap.get_cost(nx, ny) != OccupancyGrid2d.FREE:
                    continue

                wx, wy = costmap.map_to_world(nx, ny)
                dist = math.hypot(wx - rep_x, wy - rep_y)

                if dist < best_dist:
                    best_dist = dist
                    best_free = (wx, wy)

        if best_free is not None:
            return best_free

        best_toward_robot = min(
            frontier_points,
            key=lambda p: math.hypot(p[0] - self.robot_x, p[1] - self.robot_y)
        )

        try:
            mx, my = costmap.world_to_map(best_toward_robot[0], best_toward_robot[1])
            free_mx, free_my = self.find_nearest_free(mx, my, costmap)
            return costmap.map_to_world(free_mx, free_my)
        except Exception:
            return best_toward_robot

    def select_frontier(self, frontiers: List[Frontier]) -> Optional[Frontier]:
        ranked: List[Frontier] = []

        mode_to_use = self.selection_mode
        if self.failure_count >= self.failure_count_before_fallback:
            mode_to_use = self.failure_fallback_mode

        for f in frontiers:
            if self.have_failed_goal:
                reject_dist = math.hypot(
                    f.goal_x - self.failed_goal_x,
                    f.goal_y - self.failed_goal_y
                )
                if reject_dist < self.failed_goal_reject_radius:
                    continue

            dist = math.hypot(f.goal_x - self.robot_x, f.goal_y - self.robot_y)

            if mode_to_use == 'nearest':
                f.score = -dist
            elif mode_to_use == 'farthest':
                f.score = dist
            elif mode_to_use == 'largest':
                f.score = float(f.size)
            else:
                inv_dist = 1.0 / max(dist, 1e-3)
                size_term = math.sqrt(float(f.size))
                f.score = self.distance_weight * inv_dist + self.size_weight * size_term

            ranked.append(f)

        if not ranked:
            return None

        ranked.sort(key=lambda x: x.score, reverse=True)

        rank = min(self.goal_rank, len(ranked) - 1)
        return ranked[rank]

    def publish_goal(self, frontier: Frontier):
        goal = PoseStamped()
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.header.frame_id = self.map_msg.header.frame_id if self.map_msg else 'map'
        goal.pose.position.x = frontier.goal_x
        goal.pose.position.y = frontier.goal_y
        goal.pose.position.z = self.goal_z
        goal.pose.orientation.w = 1.0

        self.goal_pub.publish(goal)

        self.get_logger().info(
            f'Published frontier goal | '
            f'x={frontier.goal_x:.2f} y={frontier.goal_y:.2f} z={self.goal_z:.2f} '
            f'size={frontier.size} score={frontier.score:.3f}'
        )

    def publish_markers_msg(self, frontiers: List[Frontier], best: Frontier):
        marker = Marker()
        marker.header.frame_id = self.map_msg.header.frame_id if self.map_msg else 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_centroids'
        marker.id = 0
        marker.type = Marker.SPHERE_LIST
        marker.action = Marker.ADD
        marker.scale.x = 0.20
        marker.scale.y = 0.20
        marker.scale.z = 0.20
        marker.color.a = 1.0
        marker.color.r = 0.0
        marker.color.g = 1.0
        marker.color.b = 0.0

        for f in frontiers:
            p = Point()
            p.x = f.x
            p.y = f.y
            p.z = 0.05
            marker.points.append(p)

        self.marker_pub.publish(marker)

        best_marker = Marker()
        best_marker.header.frame_id = marker.header.frame_id
        best_marker.header.stamp = marker.header.stamp
        best_marker.ns = 'best_frontier'
        best_marker.id = 1
        best_marker.type = Marker.SPHERE
        best_marker.action = Marker.ADD
        best_marker.pose.position.x = best.goal_x
        best_marker.pose.position.y = best.goal_y
        best_marker.pose.position.z = 0.08
        best_marker.pose.orientation.w = 1.0
        best_marker.scale.x = 0.35
        best_marker.scale.y = 0.35
        best_marker.scale.z = 0.35
        best_marker.color.a = 1.0
        best_marker.color.r = 1.0
        best_marker.color.g = 0.0
        best_marker.color.b = 0.0

        self.marker_pub.publish(best_marker)

    def publish_empty_marker(self):
        marker = Marker()
        marker.header.frame_id = self.map_msg.header.frame_id if self.map_msg else 'map'
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'frontier_centroids'
        marker.id = 0
        marker.action = Marker.DELETEALL
        self.marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)
    node = FrontierSelector()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

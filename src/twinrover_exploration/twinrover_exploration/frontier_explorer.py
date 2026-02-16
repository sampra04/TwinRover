#!/usr/bin/env python3
import math
from collections import deque
from typing import List, Optional, Tuple

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.time import Time

from builtin_interfaces.msg import Duration as RosDuration
from geometry_msgs.msg import Point, PoseStamped
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import OccupancyGrid, Odometry
from std_msgs.msg import Bool, String
from visualization_msgs.msg import Marker, MarkerArray


GridCell = Tuple[int, int]


class FrontierExplorer(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer')

        self.declare_parameter('update_rate_hz', 1.0)
        self.declare_parameter('min_frontier_cells', 12)
        self.declare_parameter('unknown_gain_weight', 1.0)
        self.declare_parameter('distance_weight', 0.7)
        self.declare_parameter('risk_weight', 0.8)
        self.declare_parameter('goal_timeout_s', 45.0)
        self.declare_parameter('blacklist_duration_s', 90.0)
        self.declare_parameter('completion_empty_cycles', 5)
        self.declare_parameter('safety_clearance_m', 0.30)
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('manual_mode_topic', '/exploration/manual_mode')
        self.declare_parameter('status_topic', '/exploration/status')
        self.declare_parameter('frontier_marker_topic', '/exploration/frontiers')

        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.min_frontier_cells = int(self.get_parameter('min_frontier_cells').value)
        self.w_info = float(self.get_parameter('unknown_gain_weight').value)
        self.w_dist = float(self.get_parameter('distance_weight').value)
        self.w_risk = float(self.get_parameter('risk_weight').value)
        self.goal_timeout = Duration(seconds=float(self.get_parameter('goal_timeout_s').value))
        self.blacklist_duration = Duration(seconds=float(self.get_parameter('blacklist_duration_s').value))
        self.completion_empty_cycles = int(self.get_parameter('completion_empty_cycles').value)
        self.safety_clearance_m = float(self.get_parameter('safety_clearance_m').value)

        map_topic = str(self.get_parameter('map_topic').value)
        manual_mode_topic = str(self.get_parameter('manual_mode_topic').value)
        status_topic = str(self.get_parameter('status_topic').value)
        marker_topic = str(self.get_parameter('frontier_marker_topic').value)

        self.map_sub = self.create_subscription(OccupancyGrid, map_topic, self.on_map, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.manual_mode_sub = self.create_subscription(Bool, manual_mode_topic, self.on_manual_mode, 10)

        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.marker_pub = self.create_publisher(MarkerArray, marker_topic, 10)

        self.nav_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')

        self.map_msg: Optional[OccupancyGrid] = None
        self.current_odom_xy: Optional[Tuple[float, float]] = None
        self.current_goal_handle = None
        self.current_goal_cell: Optional[GridCell] = None
        self.current_goal_sent_time: Optional[Time] = None
        self.manual_mode = False
        self.finished = False
        self.empty_frontier_cycles = 0
        self.goal_blacklist = {}

        period = 1.0 / max(0.1, self.update_rate_hz)
        self.timer = self.create_timer(period, self.on_timer)

        self.get_logger().info('Frontier explorer initialized.')

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def on_odom(self, msg: Odometry) -> None:
        self.current_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_manual_mode(self, msg: Bool) -> None:
        was_manual = self.manual_mode
        self.manual_mode = msg.data
        if self.manual_mode and not was_manual:
            self.publish_status('manual_override')
            self.cancel_active_goal()
        if (not self.manual_mode) and was_manual:
            self.publish_status('exploring')

    def publish_status(self, text: str) -> None:
        msg = String()
        msg.data = text
        self.status_pub.publish(msg)

    def on_timer(self) -> None:
        self.cleanup_blacklist()

        if self.finished:
            return
        if self.manual_mode:
            return
        if self.map_msg is None:
            self.publish_status('waiting_for_map')
            return

        if not self.nav_client.server_is_ready():
            self.publish_status('waiting_for_nav2')
            return

        if self.current_goal_handle is not None:
            if self.current_goal_sent_time and (self.get_clock().now() - self.current_goal_sent_time) > self.goal_timeout:
                self.get_logger().warn('Goal timeout. Cancelling and replanning.')
                self.publish_status('goal_timeout_replan')
                self.blacklist_current_goal()
                self.cancel_active_goal()
            return

        frontiers = self.detect_frontiers(self.map_msg)
        self.publish_frontier_markers(frontiers)

        if not frontiers:
            self.empty_frontier_cycles += 1
            self.publish_status(f'no_frontier_cycle_{self.empty_frontier_cycles}')
            if self.empty_frontier_cycles >= self.completion_empty_cycles:
                self.publish_status('complete')
                self.finished = True
                self.get_logger().info('Exploration complete: no valid frontiers remain.')
            return

        self.empty_frontier_cycles = 0
        best = self.select_best_frontier(frontiers)
        if best is None:
            self.publish_status('no_valid_frontier')
            return

        self.send_nav_goal(best)

    def cleanup_blacklist(self) -> None:
        now = self.get_clock().now()
        expired = [cell for cell, ts in self.goal_blacklist.items() if now >= ts]
        for cell in expired:
            del self.goal_blacklist[cell]

    def blacklist_current_goal(self) -> None:
        if self.current_goal_cell is None:
            return
        expire = self.get_clock().now() + self.blacklist_duration
        self.goal_blacklist[self.current_goal_cell] = expire

    def cancel_active_goal(self) -> None:
        if self.current_goal_handle is not None:
            _ = self.current_goal_handle.cancel_goal_async()
        self.current_goal_handle = None
        self.current_goal_cell = None
        self.current_goal_sent_time = None

    def send_nav_goal(self, cell: GridCell) -> None:
        if self.map_msg is None:
            return

        pose = self.cell_to_pose(self.map_msg, cell)
        goal = NavigateToPose.Goal()
        goal.pose = pose

        self.current_goal_cell = cell
        self.current_goal_sent_time = self.get_clock().now()

        future = self.nav_client.send_goal_async(goal, feedback_callback=self.feedback_callback)
        future.add_done_callback(self.goal_response_callback)
        self.publish_status('navigating')

    def goal_response_callback(self, future) -> None:
        goal_handle = future.result()
        if goal_handle is None or not goal_handle.accepted:
            self.get_logger().warn('Nav2 rejected frontier goal.')
            self.blacklist_current_goal()
            self.cancel_active_goal()
            self.publish_status('goal_rejected')
            return

        self.current_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.goal_result_callback)

    def goal_result_callback(self, future) -> None:
        status_code = future.result().status if future.result() is not None else None

        if status_code == GoalStatus.STATUS_ABORTED:
            self.get_logger().warn('Goal aborted by Nav2.')
            self.blacklist_current_goal()
            self.publish_status('goal_aborted')
        elif status_code == GoalStatus.STATUS_CANCELED:
            self.get_logger().info('Goal canceled.')
            self.publish_status('goal_canceled')
        elif status_code == GoalStatus.STATUS_SUCCEEDED:
            self.publish_status('goal_reached')
        else:
            self.publish_status(f'goal_status_{status_code}')

        self.current_goal_handle = None
        self.current_goal_cell = None
        self.current_goal_sent_time = None

    def feedback_callback(self, _feedback_msg) -> None:
        pass

    def detect_frontiers(self, grid: OccupancyGrid) -> List[List[GridCell]]:
        width = grid.info.width
        height = grid.info.height
        data = grid.data

        def index(x: int, y: int) -> int:
            return y * width + x

        def is_free(x: int, y: int) -> bool:
            return data[index(x, y)] == 0

        def is_unknown(x: int, y: int) -> bool:
            return data[index(x, y)] == -1

        frontier_cells = set()
        for y in range(1, height - 1):
            for x in range(1, width - 1):
                if not is_free(x, y):
                    continue
                neighbors = [
                    (x + 1, y), (x - 1, y), (x, y + 1), (x, y - 1),
                    (x + 1, y + 1), (x - 1, y - 1), (x + 1, y - 1), (x - 1, y + 1)
                ]
                if any(is_unknown(nx, ny) for nx, ny in neighbors):
                    frontier_cells.add((x, y))

        clusters = []
        visited = set()
        for cell in frontier_cells:
            if cell in visited:
                continue
            cluster = []
            queue = deque([cell])
            visited.add(cell)
            while queue:
                cx, cy = queue.popleft()
                cluster.append((cx, cy))
                for nx, ny in [(cx + 1, cy), (cx - 1, cy), (cx, cy + 1), (cx, cy - 1)]:
                    ncell = (nx, ny)
                    if ncell in frontier_cells and ncell not in visited:
                        visited.add(ncell)
                        queue.append(ncell)
            if len(cluster) >= self.min_frontier_cells:
                clusters.append(cluster)

        filtered = []
        for cluster in clusters:
            safe_cells = [c for c in cluster if self.cell_is_safe(grid, c)]
            if len(safe_cells) >= self.min_frontier_cells:
                filtered.append(safe_cells)
        return filtered

    def cell_is_safe(self, grid: OccupancyGrid, cell: GridCell) -> bool:
        width = grid.info.width
        height = grid.info.height
        data = grid.data
        resolution = grid.info.resolution

        radius_cells = max(1, int(self.safety_clearance_m / max(1e-3, resolution)))
        cx, cy = cell

        for dy in range(-radius_cells, radius_cells + 1):
            for dx in range(-radius_cells, radius_cells + 1):
                x = cx + dx
                y = cy + dy
                if x < 0 or x >= width or y < 0 or y >= height:
                    return False
                cost = data[y * width + x]
                if cost > 50:
                    return False
        return True

    def select_best_frontier(self, frontiers: List[List[GridCell]]) -> Optional[GridCell]:
        if self.map_msg is None:
            return None

        robot_xy = self.estimate_robot_xy(self.map_msg)
        if robot_xy is None:
            robot_xy = self.cell_to_xy(self.map_msg, self.map_center_cell(self.map_msg))

        best_cell = None
        best_score = -float('inf')

        for cluster in frontiers:
            centroid = self.cluster_centroid(cluster)
            if centroid in self.goal_blacklist:
                continue

            info_gain = float(len(cluster))
            cxy = self.cell_to_xy(self.map_msg, centroid)
            dist = self.euclidean(robot_xy, cxy)
            risk = self.estimate_obstacle_proximity_penalty(self.map_msg, centroid)

            score = self.w_info * info_gain - self.w_dist * dist - self.w_risk * risk
            if score > best_score:
                best_score = score
                best_cell = centroid

        return best_cell

    def euclidean(self, a: Tuple[float, float], b: Tuple[float, float]) -> float:
        return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)

    def estimate_robot_xy(self, grid: OccupancyGrid) -> Optional[Tuple[float, float]]:
        if self.current_odom_xy is not None:
            return self.current_odom_xy
        return self.cell_to_xy(grid, self.map_center_cell(grid))

    def map_center_cell(self, grid: OccupancyGrid) -> GridCell:
        return (grid.info.width // 2, grid.info.height // 2)

    def cluster_centroid(self, cluster: List[GridCell]) -> GridCell:
        sx = sum(c[0] for c in cluster)
        sy = sum(c[1] for c in cluster)
        return (int(sx / len(cluster)), int(sy / len(cluster)))

    def estimate_obstacle_proximity_penalty(self, grid: OccupancyGrid, cell: GridCell) -> float:
        width = grid.info.width
        height = grid.info.height
        data = grid.data
        cx, cy = cell

        max_radius = 6
        penalty = 0.0

        for r in range(1, max_radius + 1):
            hits = 0
            samples = 0
            for dy in range(-r, r + 1):
                for dx in range(-r, r + 1):
                    x = cx + dx
                    y = cy + dy
                    if x < 0 or x >= width or y < 0 or y >= height:
                        continue
                    if abs(dx) != r and abs(dy) != r:
                        continue
                    samples += 1
                    if data[y * width + x] > 50:
                        hits += 1
            if samples > 0:
                penalty += float(hits) / float(samples)
        return penalty

    def cell_to_xy(self, grid: OccupancyGrid, cell: GridCell) -> Tuple[float, float]:
        x = grid.info.origin.position.x + (cell[0] + 0.5) * grid.info.resolution
        y = grid.info.origin.position.y + (cell[1] + 0.5) * grid.info.resolution
        return (x, y)

    def cell_to_pose(self, grid: OccupancyGrid, cell: GridCell) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = grid.header.frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()
        xy = self.cell_to_xy(grid, cell)
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def publish_frontier_markers(self, frontiers: List[List[GridCell]]) -> None:
        if self.map_msg is None:
            return

        marker_array = MarkerArray()
        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for i, cluster in enumerate(frontiers):
            marker = Marker()
            marker.header.frame_id = self.map_msg.header.frame_id or 'map'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'frontiers'
            marker.id = i
            marker.type = Marker.POINTS
            marker.action = Marker.ADD
            marker.scale.x = 0.08
            marker.scale.y = 0.08
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 1.0
            marker.color.a = 0.9
            marker.lifetime = RosDuration(sec=1, nanosec=0)
            for cell in cluster:
                xy = self.cell_to_xy(self.map_msg, cell)
                p = Point()
                p.x = xy[0]
                p.y = xy[1]
                p.z = 0.02
                marker.points.append(p)
            marker_array.markers.append(marker)

        self.marker_pub.publish(marker_array)


def main() -> None:
    rclpy.init()
    node = FrontierExplorer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

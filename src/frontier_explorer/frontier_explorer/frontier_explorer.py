import math
from collections import deque

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node


class FrontierExplorer(Node):
    def __init__(self):
        super().__init__("frontier_explorer")
        self.navigator = BasicNavigator()
        self.map_data = None
        self.robot_pose = None

        # Minimal tunables for a stable baseline.
        self.map_topic = "/map"
        self.odom_topic = "/odom_filtered"
        self.odom_fallback_topics = ["/odom", "/diff_drive_controller/odom"]
        self.min_cluster_size = 5
        self.min_goal_distance = 0.4
        self.heading_weight = 1.0  # meters of equivalent cost per radian
        self.goal_timeout_s = 60.0
        self.blacklist_duration_s = 30.0
        self.blacklist_radius = 0.75

        self.current_goal = None
        self.current_goal_sent_time = None
        self.blacklist = {}  # {(x, y): expiry_time_ns}
        self.no_frontier_log_period_ns = int(5.0 * 1e9)
        self.last_no_frontier_log_ns = 0

        self.create_subscription(OccupancyGrid, self.map_topic, self.map_callback, 10)
        self.create_subscription(Odometry, self.odom_topic, self.odom_callback, 10)
        for topic in self.odom_fallback_topics:
            self.create_subscription(Odometry, topic, self.odom_callback, 10)

        self.get_logger().info("Frontier Explorer (simple baseline) initialized.")

    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def cleanup_blacklist(self, now_ns):
        expired = [goal for goal, expiry in self.blacklist.items() if expiry <= now_ns]
        for goal in expired:
            del self.blacklist[goal]

    def goal_is_blacklisted(self, x, y, now_ns):
        for (bx, by), expiry in self.blacklist.items():
            if expiry > now_ns and math.hypot(x - bx, y - by) <= self.blacklist_radius:
                return True
        return False

    def detect_frontier_clusters(self, grid):
        height, width = grid.shape
        frontier_mask = np.zeros((height, width), dtype=bool)
        if height < 3 or width < 3:
            return []

        center = grid[1:-1, 1:-1]
        unknown_center = center == -1
        adjacent_free = (
            (grid[:-2, 1:-1] == 0)
            | (grid[2:, 1:-1] == 0)
            | (grid[1:-1, :-2] == 0)
            | (grid[1:-1, 2:] == 0)
        )
        frontier_mask[1:-1, 1:-1] = unknown_center & adjacent_free

        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        for y, x in np.argwhere(frontier_mask):
            y = int(y)
            x = int(x)
            if visited[y, x]:
                continue

            queue = deque([(y, x)])
            visited[y, x] = True
            sum_y = 0
            sum_x = 0
            count = 0

            while queue:
                cy, cx = queue.popleft()
                sum_y += cy
                sum_x += cx
                count += 1

                for ny in range(max(0, cy - 1), min(height, cy + 2)):
                    for nx in range(max(0, cx - 1), min(width, cx + 2)):
                        if not visited[ny, nx] and frontier_mask[ny, nx]:
                            visited[ny, nx] = True
                            queue.append((ny, nx))

            if count >= self.min_cluster_size:
                clusters.append((sum_y, sum_x, count))

        return clusters

    def choose_goal(self, clusters, resolution, origin_x, origin_y, now_ns):
        if self.robot_pose is None or not clusters:
            return None

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y
        robot_yaw = self.yaw_from_quaternion(self.robot_pose.orientation)
        best_goal = None
        best_score = float("inf")

        for sum_y, sum_x, cluster_size in clusters:
            cy = int(sum_y / cluster_size)
            cx = int(sum_x / cluster_size)
            goal_x = origin_x + cx * resolution
            goal_y = origin_y + cy * resolution

            distance = math.hypot(goal_x - robot_x, goal_y - robot_y)
            if distance < self.min_goal_distance:
                continue
            if self.goal_is_blacklisted(goal_x, goal_y, now_ns):
                continue

            goal_bearing = math.atan2(goal_y - robot_y, goal_x - robot_x)
            heading_error = abs(self.normalize_angle(goal_bearing - robot_yaw))
            score = distance + self.heading_weight * heading_error

            if score < best_score:
                best_score = score
                best_goal = (goal_x, goal_y)

        return best_goal

    def yaw_from_quaternion(self, orientation):
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        return math.atan2(siny_cosp, cosy_cosp)

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def clear_current_goal(self):
        self.current_goal = None
        self.current_goal_sent_time = None

    def blacklist_current_goal(self):
        if self.current_goal is None:
            return
        now_ns = self.get_clock().now().nanoseconds
        expiry = now_ns + int(self.blacklist_duration_s * 1e9)
        self.blacklist[self.current_goal] = expiry
        self.get_logger().warn(
            f"Blacklisting goal {self.current_goal} for {self.blacklist_duration_s:.0f}s"
        )

    def send_goal(self, goal):
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = goal[0]
        pose.pose.position.y = goal[1]
        if self.robot_pose is not None:
            # Keep current heading to avoid unnecessary rotate-in-place behavior.
            pose.pose.orientation = self.robot_pose.orientation
        else:
            pose.pose.orientation.w = 1.0

        self.current_goal = goal
        self.current_goal_sent_time = self.get_clock().now()
        self.navigator.goToPose(pose)
        self.get_logger().info(f"Sent goal: {goal}")

    def handle_active_goal(self):
        if self.current_goal is None or self.current_goal_sent_time is None:
            return

        elapsed = (self.get_clock().now() - self.current_goal_sent_time).nanoseconds / 1e9
        if elapsed > self.goal_timeout_s:
            self.get_logger().warn(f"Goal timed out after {elapsed:.1f}s")
            self.navigator.cancelTask()
            self.blacklist_current_goal()
            self.clear_current_goal()
            return

        if not self.navigator.isTaskComplete():
            return

        result = self.navigator.getResult()
        if result == TaskResult.SUCCEEDED:
            self.get_logger().info(f"Goal reached: {self.current_goal}")
        else:
            self.get_logger().warn(f"Goal failed with result={result}")
            self.blacklist_current_goal()
        self.clear_current_goal()

    def explore(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.2)

            if self.map_data is None:
                continue

            now_ns = self.get_clock().now().nanoseconds
            self.cleanup_blacklist(now_ns)
            self.handle_active_goal()

            if self.current_goal is not None:
                continue

            map_msg = self.map_data
            width = map_msg.info.width
            height = map_msg.info.height
            resolution = map_msg.info.resolution
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y

            try:
                grid = np.frombuffer(map_msg.data, dtype=np.int8).reshape((height, width))
            except TypeError:
                grid = np.asarray(map_msg.data, dtype=np.int8).reshape((height, width))

            clusters = self.detect_frontier_clusters(grid)
            goal = self.choose_goal(clusters, resolution, origin_x, origin_y, now_ns)
            if goal is None:
                if now_ns - self.last_no_frontier_log_ns >= self.no_frontier_log_period_ns:
                    self.get_logger().info("No valid frontier found.")
                    self.last_no_frontier_log_ns = now_ns
                continue

            self.send_goal(goal)


def main():
    rclpy.init()
    explorer = FrontierExplorer()
    try:
        explorer.explore()
    finally:
        explorer.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

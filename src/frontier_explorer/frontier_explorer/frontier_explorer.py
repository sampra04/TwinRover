'''
import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import random

class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')
        self.navigator = BasicNavigator()
        self.map_data = None

        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

    def map_callback(self, msg):
        self.map_data = msg

    def find_frontier(self):
        if self.map_data is None:
            return None

        width = self.map_data.info.width
        height = self.map_data.info.height
        resolution = self.map_data.info.resolution
        origin = self.map_data.info.origin

        data = np.array(self.map_data.data).reshape((height, width))

        unknown = np.where(data == -1)
        if len(unknown[0]) == 0:
            return None

        idx = random.randint(0, len(unknown[0]) - 1)
        y = unknown[0][idx]
        x = unknown[1][idx]

        world_x = origin.position.x + x * resolution
        world_y = origin.position.y + y * resolution

        return world_x, world_y

    def explore(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)

            goal = self.find_frontier()
            if goal is None:
                continue

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = goal[0]
            pose.pose.position.y = goal[1]
            pose.pose.orientation.w = 1.0

            self.get_logger().info(f"Sending goal: {goal}")
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)

def main():
    rclpy.init()
    explorer = FrontierExplorer()
    explorer.explore()

if __name__ == '__main__':
    main()
'''

import rclpy
from rclpy.node import Node
from nav2_simple_commander.robot_navigator import BasicNavigator
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
import numpy as np
import math
from collections import deque


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.navigator = BasicNavigator()
        self.map_data = None
        self.robot_pose = None
        self.alpha = 1.5
        self.beta = 2.0
        self.no_frontier_log_period_ns = int(5.0 * 1e9)
        self.last_no_frontier_log_ns = 0

        # Subscribe to SLAM map
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        # Subscribe to odometry (SLAM Toolbox compatible)
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.get_logger().info("Frontier Explorer Initialized")

    # ==============================
    # Callbacks
    # ==============================
    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    # ==============================
    # Frontier Detection
    # ==============================
    def detect_frontiers(self, data):
        height, width = data.shape
        frontier_mask = np.zeros((height, width), dtype=bool)
        if height < 3 or width < 3:
            return frontier_mask

        center = data[1:-1, 1:-1]
        unknown_center = (center == -1)
        adjacent_free = (
            (data[:-2, 1:-1] == 0) |
            (data[2:, 1:-1] == 0) |
            (data[1:-1, :-2] == 0) |
            (data[1:-1, 2:] == 0)
        )
        frontier_mask[1:-1, 1:-1] = unknown_center & adjacent_free
        return frontier_mask

    # ==============================
    # Cluster Frontiers (BFS)
    # ==============================
    def cluster_frontiers(self, frontier_mask):
        visited = np.zeros_like(frontier_mask, dtype=bool)
        clusters = []

        height, width = frontier_mask.shape
        frontier_points = np.argwhere(frontier_mask)
        for y, x in frontier_points:
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

                y_start = max(0, cy - 1)
                y_end = min(height, cy + 2)
                x_start = max(0, cx - 1)
                x_end = min(width, cx + 2)
                for ny in range(y_start, y_end):
                    for nx in range(x_start, x_end):
                        if not visited[ny, nx] and frontier_mask[ny, nx]:
                            visited[ny, nx] = True
                            queue.append((ny, nx))

            clusters.append((sum_y, sum_x, count))

        return clusters

    # ==============================
    # Choose Best Frontier
    # ==============================
    def choose_best_frontier(self, clusters, resolution, origin_x, origin_y):
        if not clusters:
            return None

        if self.robot_pose is None:
            return None

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        best_score = float('inf')
        best_goal = None

        for sum_y, sum_x, cluster_size in clusters:
            cy = int(sum_y / cluster_size)
            cx = int(sum_x / cluster_size)

            world_x = origin_x + cx * resolution
            world_y = origin_y + cy * resolution

            dx = world_x - robot_x
            dy = world_y - robot_y
            distance = math.hypot(dx, dy)
            score = self.alpha * distance - self.beta * cluster_size

            if score < best_score:
                best_score = score
                best_goal = (world_x, world_y)

        return best_goal

    # ==============================
    # Main Exploration Loop
    # ==============================
    def explore(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=1.0)

            if self.map_data is None:
                continue

            map_msg = self.map_data
            width = map_msg.info.width
            height = map_msg.info.height
            resolution = map_msg.info.resolution
            origin_x = map_msg.info.origin.position.x
            origin_y = map_msg.info.origin.position.y
            try:
                data = np.frombuffer(map_msg.data, dtype=np.int8).reshape((height, width))
            except TypeError:
                data = np.asarray(map_msg.data, dtype=np.int8).reshape((height, width))

            # Debug (optional)
            # self.get_logger().info(
            #     f"Unknown: {np.sum(data == -1)}, Free: {np.sum(data == 0)}"
            # )

            frontier_mask = self.detect_frontiers(data)
            clusters = self.cluster_frontiers(frontier_mask)

            goal = self.choose_best_frontier(clusters, resolution, origin_x, origin_y)

            if goal is None:
                now_ns = self.get_clock().now().nanoseconds
                if now_ns - self.last_no_frontier_log_ns >= self.no_frontier_log_period_ns:
                    self.get_logger().info("No frontier found.")
                    self.last_no_frontier_log_ns = now_ns
                continue

            pose = PoseStamped()
            pose.header.frame_id = 'map'
            pose.pose.position.x = goal[0]
            pose.pose.position.y = goal[1]
            pose.pose.orientation.w = 1.0

            self.get_logger().info(f"Sending goal: {goal}")
            self.navigator.goToPose(pose)

            while not self.navigator.isTaskComplete():
                rclpy.spin_once(self, timeout_sec=0.5)

            self.get_logger().info("Goal complete.")


def main():
    rclpy.init()
    explorer = FrontierExplorer()
    explorer.explore()


if __name__ == '__main__':
    main()

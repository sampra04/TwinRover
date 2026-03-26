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
from nav_msgs.msg import OccupancyGrid, Odometry, Path
import numpy as np
import math
from collections import deque


class FrontierExplorer(Node):

    def __init__(self):
        super().__init__('frontier_explorer')

        self.navigator = BasicNavigator()
        self.map_data = None
        self.robot_pose = None

        # Scoring weights
        self.alpha = 1.5       # distance penalty
        self.beta = 2.0        # frontier size reward
        self.gamma = 3.0       # local radius bonus
        self.local_radius = 4.0  # metres - tune to your environment

        # Add to __init__
        self.min_goal_distance = 0.5      # metres - don't send goals closer than this
        self.max_cluster_size = 500000    # ignore unrealistically large clusters
        self.visited_goals = []           # track recent goals to avoid revisiting
        self.visited_goal_radius = 0.5    # metres - how close counts as "already visited"

        self.goal_lookahead_fraction = 0.7  # start planning next goal at 70% distance travelled
        self.next_goal = None
        self.next_goal_time = None          # timestamp when next_goal was pre-planned
        self.next_goal_max_age_sec = 10.0   # discard pre-planned goal older than this
        self.current_goal = None
        self.current_goal_distance = None

        # Goal timeout: scaled by distance at runtime
        self.nav_speed_mps = 0.25        # match max_vel_x in nav2_explore.yaml
        self.goal_timeout_margin = 2.0   # safety factor over ideal travel time
        self.goal_timeout_min_sec = 45.0 # floor timeout regardless of distance
        self.goal_timeout_sec = self.goal_timeout_min_sec  # updated per goal
        self.current_goal_start_time = None

        # Blacklist: timed-out goals are avoided for this many seconds
        self.blacklist = {}           # {(x, y): expiry_time_ns}
        self.blacklist_duration_s = 120.0
        self.blacklist_radius = 1.0   # metres - how close to a blacklisted goal counts


        # Logging throttle
        self.no_frontier_log_period_ns = int(5.0 * 1e9)
        self.last_no_frontier_log_ns = 0

        # Subscriptions
        self.create_subscription(
            OccupancyGrid,
            '/map',
            self.map_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.create_subscription(
            Path,
            '/plan',
            self.plan_callback,
            10
        )
        self._current_goal_active = False  # guards against stale plan updates
        self._path_captured = False        # True after first plan for current goal received

        self.get_logger().info("WFD Frontier Explorer Initialized")

    # ==============================
    # Callbacks
    # ==============================
    def map_callback(self, msg):
        self.map_data = msg

    def odom_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def plan_callback(self, msg):
        if not self._current_goal_active or self._path_captured or not msg.poses:
            return
        self._path_captured = True  # lock out subsequent replans for this goal

        # Sum distances between consecutive waypoints for true path length
        path_length = 0.0
        poses = msg.poses
        for i in range(1, len(poses)):
            dx = poses[i].pose.position.x - poses[i-1].pose.position.x
            dy = poses[i].pose.position.y - poses[i-1].pose.position.y
            path_length += math.hypot(dx, dy)

        if path_length > 0.0:
            self.current_goal_distance = path_length
            self.goal_timeout_sec = max(
                self.goal_timeout_min_sec,
                (path_length / self.nav_speed_mps) * self.goal_timeout_margin
            )
            self.get_logger().info(
                f"Path distance updated: {path_length:.1f}m "
                f"→ timeout {self.goal_timeout_sec:.0f}s"
            )

    # ==============================
    # Wavefront Frontier Detection
    # ==============================
    def wfd(self, data, resolution, origin_x, origin_y):

        if self.robot_pose is None:
            return []

        height, width = data.shape

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        rx = int((robot_x - origin_x) / resolution)
        ry = int((robot_y - origin_y) / resolution)

        if not (0 <= ry < height and 0 <= rx < width):
            return []

        # Adaptive start-cell search: progressively widen radius, always
        # picking the nearest free cell (not the top-left biased one).
        start = None
        for search_radius in (5, 10, 15):
            candidates = []
            for dr in range(-search_radius, search_radius + 1):
                for dc in range(-search_radius, search_radius + 1):
                    nr, nc = ry + dr, rx + dc
                    if (0 <= nr < height and 0 <= nc < width
                            and data[nr, nc] == 0):
                        candidates.append((dr * dr + dc * dc, nr, nc))
            if candidates:
                candidates.sort()
                _, sr, sc = candidates[0]
                start = (sr, sc)
                break

        if start is None:
            self.get_logger().warn("WFD: No free cell found within 15-cell radius of robot")
            return []

        visited = np.zeros_like(data, dtype=bool)
        frontier_visited = np.zeros_like(data, dtype=bool)

        clusters = []
        queue = deque()
        queue.append(start)
        visited[start[0], start[1]] = True

        while queue:
            cy, cx = queue.popleft()

            for ny in range(max(0, cy - 1), min(height, cy + 2)):
                for nx in range(max(0, cx - 1), min(width, cx + 2)):

                    if visited[ny, nx]:
                        continue

                    if data[ny, nx] == 0:
                        visited[ny, nx] = True
                        queue.append((ny, nx))

                    elif data[ny, nx] == -1:
                        if frontier_visited[ny, nx]:
                            continue

                        # BFS to cluster this frontier
                        fqueue = deque()
                        fqueue.append((ny, nx))
                        frontier_visited[ny, nx] = True

                        sum_y, sum_x, count = 0, 0, 0

                        while fqueue:
                            fy, fx = fqueue.popleft()
                            sum_y += fy
                            sum_x += fx
                            count += 1

                            for fny in range(max(0, fy - 1), min(height, fy + 2)):
                                for fnx in range(max(0, fx - 1), min(width, fx + 2)):
                                    if (0 <= fny < height and 0 <= fnx < width
                                            and not frontier_visited[fny, fnx]
                                            and data[fny, fnx] == -1):
                                        frontier_visited[fny, fnx] = True
                                        fqueue.append((fny, fnx))

                        clusters.append((sum_y, sum_x, count))

        return clusters

    # ==============================
    # Choose Best Frontier
    # ==============================
    def choose_best_frontier(self, clusters, resolution, origin_x, origin_y):
        if not clusters or self.robot_pose is None:
            return None

        robot_x = self.robot_pose.position.x
        robot_y = self.robot_pose.position.y

        sizes = [c[2] for c in clusters]
        self.get_logger().info(
            f"Clusters found: {len(clusters)}, "
            f"top sizes: {sorted(sizes, reverse=True)[:5]}"
        )

        # Progressive filter relaxation: try full visited radius, then half if nothing survives.
        candidates = []
        for attempt, visited_radius in enumerate(
                [self.visited_goal_radius, self.visited_goal_radius / 2.0]):

            local_candidates = []
            global_candidates = []

            for sum_y, sum_x, cluster_size in clusters:

                # Filter noise AND giant false clusters
                if cluster_size < 3 or cluster_size > self.max_cluster_size:
                    continue

                cy = int(sum_y / cluster_size)
                cx = int(sum_x / cluster_size)
                world_x = origin_x + cx * resolution
                world_y = origin_y + cy * resolution
                distance = math.hypot(world_x - robot_x, world_y - robot_y)

                # Skip goals too close to the robot
                if distance < self.min_goal_distance:
                    continue

                # Skip recently visited goals (relaxed on second attempt)
                too_close_to_visited = any(
                    math.hypot(world_x - vx, world_y - vy) < visited_radius
                    for vx, vy in self.visited_goals
                )
                if too_close_to_visited:
                    continue

                # Skip blacklisted goals (timed-out positions)
                now_ns = self.get_clock().now().nanoseconds
                if any(
                    math.hypot(world_x - bx, world_y - by) < self.blacklist_radius
                    and now_ns < expiry
                    for (bx, by), expiry in self.blacklist.items()
                ):
                    continue

                if distance <= self.local_radius:
                    local_candidates.append((world_x, world_y, distance, cluster_size))
                else:
                    global_candidates.append((world_x, world_y, distance, cluster_size))

            candidates = local_candidates if local_candidates else global_candidates

            if candidates:
                if attempt > 0:
                    self.get_logger().info(
                        f"Filter relaxed (visited_radius={visited_radius:.2f}m) "
                        f"— found {len(candidates)} candidate(s)."
                    )
                break  # found candidates, stop relaxing

        if not candidates:
            self.get_logger().warn(
                f"No valid candidates after filter relaxation. clusters={len(clusters)}, "
                f"check max_cluster_size ({self.max_cluster_size}) "
                f"and min_goal_distance ({self.min_goal_distance})"
            )
            return None

        best_score = float('inf')
        best_goal = None
        best_cluster_size = None

        for world_x, world_y, distance, cluster_size in candidates:
            score = self.alpha * distance - self.beta * cluster_size
            if distance <= self.local_radius:
                score -= self.gamma

            if score < best_score:
                best_score = score
                best_goal = (world_x, world_y)
                best_cluster_size = cluster_size

        if best_goal is not None:
            self.get_logger().info(
                f"Chosen frontier: {best_goal}, cluster size: {best_cluster_size}"
            )

        return best_goal

    # ==============================
    # Main Exploration Loop
    # ==============================
    def explore(self):
        while rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0.1)

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

            # If no active goal, pick one and send it
            if self.current_goal is None:

                # Use pre-planned next goal if available and not stale
                if self.next_goal is not None:
                    age = (
                        (self.get_clock().now() - self.next_goal_time).nanoseconds / 1e9
                        if self.next_goal_time is not None else float('inf')
                    )
                    if age <= self.next_goal_max_age_sec:
                        goal = self.next_goal
                        self.get_logger().info(f"Using pre-planned next goal (age {age:.1f}s).")
                    else:
                        goal = None
                        self.get_logger().info(f"Discarding stale pre-planned goal (age {age:.1f}s).")
                    self.next_goal = None
                    self.next_goal_time = None
                else:
                    clusters = self.wfd(data, resolution, origin_x, origin_y)
                    goal = self.choose_best_frontier(clusters, resolution, origin_x, origin_y)

                if goal is None:
                    now_ns = self.get_clock().now().nanoseconds
                    if now_ns - self.last_no_frontier_log_ns >= self.no_frontier_log_period_ns:
                        self.get_logger().info("No frontier found.")
                        self.last_no_frontier_log_ns = now_ns
                    continue

                # Record goal and initial distance
                self.current_goal = goal
                if self.robot_pose is not None:
                    self.current_goal_distance = math.hypot(
                        goal[0] - self.robot_pose.position.x,
                        goal[1] - self.robot_pose.position.y
                    )

                self.visited_goals.append(goal)
                if len(self.visited_goals) > 50:
                    self.visited_goals.pop(0)

                pose = PoseStamped()
                pose.header.frame_id = 'map'
                pose.pose.position.x = goal[0]
                pose.pose.position.y = goal[1]
                pose.pose.orientation.w = 1.0

                self.get_logger().info(f"Sending goal: {goal}")
                self.navigator.goToPose(pose)
                self._current_goal_active = True
                self._path_captured = False
                self.current_goal_start_time = self.get_clock().now()
                if self.current_goal_distance is not None:
                    self.goal_timeout_sec = max(
                        self.goal_timeout_min_sec,
                        (self.current_goal_distance / self.nav_speed_mps) * self.goal_timeout_margin
                    )
                else:
                    self.goal_timeout_sec = self.goal_timeout_min_sec
                self.get_logger().info(
                    f"Goal timeout: {self.goal_timeout_sec:.0f}s "
                    f"(distance: {self.current_goal_distance:.1f}m)"
                )

            # Active goal — check timeout, then check completion
            if self.current_goal_start_time is not None:
                elapsed = (self.get_clock().now() - self.current_goal_start_time).nanoseconds / 1e9
                if elapsed > self.goal_timeout_sec:
                    self.get_logger().warn(
                        f"Goal timeout after {elapsed:.1f}s — blacklisting and replanning."
                    )
                    self.navigator.cancelTask()
                    # Blacklist the failed goal so it won't be re-selected
                    if self.current_goal is not None:
                        expiry = (self.get_clock().now().nanoseconds
                                  + int(self.blacklist_duration_s * 1e9))
                        self.blacklist[self.current_goal] = expiry
                        self.get_logger().info(
                            f"Blacklisted {self.current_goal} for {self.blacklist_duration_s:.0f}s"
                        )
                    self._current_goal_active = False
                    self._path_captured = False
                    self.current_goal = None
                    self.current_goal_distance = None
                    self.current_goal_start_time = None
                    self.next_goal = None
                    self.next_goal_time = None
                    continue

            if self.navigator.isTaskComplete():
                self.get_logger().info("Goal complete.")
                self._current_goal_active = False
                self._path_captured = False
                self.current_goal = None
                self.current_goal_distance = None
                self.current_goal_start_time = None
                rclpy.spin_once(self, timeout_sec=0.5)
                continue

            # Check if we've hit 70% of the way to the goal
            if (self.next_goal is None
                    and self.current_goal is not None
                    and self.current_goal_distance is not None
                    and self.robot_pose is not None):

                remaining = math.hypot(
                    self.current_goal[0] - self.robot_pose.position.x,
                    self.current_goal[1] - self.robot_pose.position.y
                )
                fraction_remaining = remaining / self.current_goal_distance

                if fraction_remaining <= (1.0 - self.goal_lookahead_fraction):
                    self.get_logger().info(
                        f"70% complete — pre-planning next goal "
                        f"(remaining: {remaining:.2f}m)"
                    )
                    clusters = self.wfd(data, resolution, origin_x, origin_y)
                    self.next_goal = self.choose_best_frontier(
                        clusters, resolution, origin_x, origin_y
                    )
                    if self.next_goal:
                        self.next_goal_time = self.get_clock().now()
                        self.get_logger().info(f"Next goal pre-planned: {self.next_goal}")
                    else:
                        self.next_goal_time = None
                        self.get_logger().info("No next goal found during pre-planning.")

def main():
    rclpy.init()
    explorer = FrontierExplorer()
    explorer.explore()


if __name__ == '__main__':
    main()
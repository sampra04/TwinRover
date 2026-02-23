from typing import Dict, Optional, Tuple

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.duration import Duration
from rclpy.node import Node

from frontier_explorer.frontier_logic import (
    GridCell,
    GridMap,
    cell_to_xy,
    cleanup_blacklist_entries,
    detect_frontiers,
    select_best_frontier,
)


class FrontierExplorerHardened(Node):
    def __init__(self) -> None:
        super().__init__('frontier_explorer_hardened')

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
        self.declare_parameter('odom_topic', '/odom')

        self.update_rate_hz = float(self.get_parameter('update_rate_hz').value)
        self.min_frontier_cells = int(self.get_parameter('min_frontier_cells').value)
        self.w_info = float(self.get_parameter('unknown_gain_weight').value)
        self.w_dist = float(self.get_parameter('distance_weight').value)
        self.w_risk = float(self.get_parameter('risk_weight').value)
        self.goal_timeout = Duration(seconds=float(self.get_parameter('goal_timeout_s').value))
        self.blacklist_duration_s = float(self.get_parameter('blacklist_duration_s').value)
        self.completion_empty_cycles = int(self.get_parameter('completion_empty_cycles').value)
        self.safety_clearance_m = float(self.get_parameter('safety_clearance_m').value)
        self.map_topic = str(self.get_parameter('map_topic').value)
        self.odom_topic = str(self.get_parameter('odom_topic').value)

        self.navigator = BasicNavigator()
        self.map_msg: Optional[OccupancyGrid] = None
        self.current_odom_xy: Optional[Tuple[float, float]] = None

        self.current_goal_cell: Optional[GridCell] = None
        self.current_goal_sent_ns: Optional[int] = None
        self.goal_blacklist: Dict[GridCell, float] = {}
        self.empty_frontier_cycles = 0
        self.finished = False

        self.create_subscription(OccupancyGrid, self.map_topic, self.on_map, 10)
        self.create_subscription(Odometry, self.odom_topic, self.on_odom, 20)

        period = 1.0 / max(0.1, self.update_rate_hz)
        self.create_timer(period, self.on_timer)
        self.get_logger().info('Hardened frontier explorer initialized.')

    def on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def on_odom(self, msg: Odometry) -> None:
        self.current_odom_xy = (msg.pose.pose.position.x, msg.pose.pose.position.y)

    def on_timer(self) -> None:
        self.cleanup_blacklist()

        if self.finished or self.map_msg is None:
            return

        if self.current_goal_cell is not None:
            if self.current_goal_sent_ns is not None:
                elapsed_ns = self.get_clock().now().nanoseconds - self.current_goal_sent_ns
                if elapsed_ns > self.goal_timeout.nanoseconds:
                    self.get_logger().warn('Goal timeout. Canceling and replanning.')
                    self.blacklist_current_goal()
                    self.navigator.cancelTask()
                    self.clear_active_goal()
                    return

            if self.navigator.isTaskComplete():
                result = self.navigator.getResult()
                if result != TaskResult.SUCCEEDED:
                    self.blacklist_current_goal()
                    self.get_logger().warn(f'Goal failed with result={result}. Replanning.')
                else:
                    self.get_logger().info('Goal reached.')
                self.clear_active_goal()
            return

        grid_map = self.to_grid_map(self.map_msg)
        frontiers = detect_frontiers(grid_map, self.min_frontier_cells, self.safety_clearance_m)

        if not frontiers:
            self.empty_frontier_cycles += 1
            if self.empty_frontier_cycles >= self.completion_empty_cycles:
                self.finished = True
                self.get_logger().info('Exploration complete: no valid frontiers remain.')
            return

        self.empty_frontier_cycles = 0

        robot_xy = self.current_odom_xy
        if robot_xy is None:
            robot_xy = cell_to_xy(grid_map, (grid_map.width // 2, grid_map.height // 2))

        goal_cell = select_best_frontier(
            grid_map,
            frontiers,
            robot_xy,
            self.goal_blacklist,
            self.w_info,
            self.w_dist,
            self.w_risk,
        )
        if goal_cell is None:
            return

        self.send_goal(goal_cell)

    def send_goal(self, cell: GridCell) -> None:
        if self.map_msg is None:
            return

        goal_pose = self.cell_to_pose(self.map_msg, cell)
        self.current_goal_cell = cell
        self.current_goal_sent_ns = self.get_clock().now().nanoseconds
        self.get_logger().info(f'Sending frontier goal cell={cell}')
        self.navigator.goToPose(goal_pose)

    def cell_to_pose(self, grid: OccupancyGrid, cell: GridCell) -> PoseStamped:
        pose = PoseStamped()
        pose.header.frame_id = grid.header.frame_id or 'map'
        pose.header.stamp = self.get_clock().now().to_msg()

        xy = cell_to_xy(self.to_grid_map(grid), cell)
        pose.pose.position.x = xy[0]
        pose.pose.position.y = xy[1]
        pose.pose.position.z = 0.0
        pose.pose.orientation.w = 1.0
        return pose

    def blacklist_current_goal(self) -> None:
        if self.current_goal_cell is None:
            return
        now_s = float(self.get_clock().now().nanoseconds) / 1e9
        self.goal_blacklist[self.current_goal_cell] = now_s + self.blacklist_duration_s

    def cleanup_blacklist(self) -> None:
        now_s = float(self.get_clock().now().nanoseconds) / 1e9
        expired = cleanup_blacklist_entries(self.goal_blacklist, now_s)
        for cell in expired:
            del self.goal_blacklist[cell]

    def clear_active_goal(self) -> None:
        self.current_goal_cell = None
        self.current_goal_sent_ns = None

    def to_grid_map(self, msg: OccupancyGrid) -> GridMap:
        return GridMap(
            width=msg.info.width,
            height=msg.info.height,
            resolution=msg.info.resolution,
            origin_x=msg.info.origin.position.x,
            origin_y=msg.info.origin.position.y,
            data=msg.data,
        )


def main() -> None:
    rclpy.init()
    node = FrontierExplorerHardened()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

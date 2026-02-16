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

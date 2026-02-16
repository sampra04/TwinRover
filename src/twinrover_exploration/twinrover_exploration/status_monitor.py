#!/usr/bin/env python3
import math

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from std_msgs.msg import String


class StatusMonitor(Node):
    def __init__(self) -> None:
        super().__init__('status_monitor')

        self.declare_parameter('status_topic', '/exploration/status')
        self.declare_parameter('heartbeat_topic', '/exploration/heartbeat')
        self.declare_parameter('heartbeat_rate_hz', 1.0)
        self.declare_parameter('stuck_timeout_s', 20.0)
        self.declare_parameter('no_progress_radius_m', 0.20)

        self.status_topic = str(self.get_parameter('status_topic').value)
        self.heartbeat_topic = str(self.get_parameter('heartbeat_topic').value)
        self.heartbeat_rate_hz = float(self.get_parameter('heartbeat_rate_hz').value)
        self.stuck_timeout_s = float(self.get_parameter('stuck_timeout_s').value)
        self.no_progress_radius_m = float(self.get_parameter('no_progress_radius_m').value)

        self.latest_status = 'init'
        self.last_progress_time = self.get_clock().now()
        self.last_pose = None

        self.status_sub = self.create_subscription(String, self.status_topic, self.on_status, 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.on_odom, 20)
        self.heartbeat_pub = self.create_publisher(String, self.heartbeat_topic, 10)

        period = 1.0 / max(0.1, self.heartbeat_rate_hz)
        self.timer = self.create_timer(period, self.publish_heartbeat)

    def on_status(self, msg: String) -> None:
        self.latest_status = msg.data

    def on_odom(self, msg: Odometry) -> None:
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if self.last_pose is None:
            self.last_pose = (x, y)
            self.last_progress_time = self.get_clock().now()
            return

        dist = math.hypot(x - self.last_pose[0], y - self.last_pose[1])
        if dist > self.no_progress_radius_m:
            self.last_pose = (x, y)
            self.last_progress_time = self.get_clock().now()

    def publish_heartbeat(self) -> None:
        elapsed = (self.get_clock().now() - self.last_progress_time).nanoseconds / 1e9
        stuck = elapsed > self.stuck_timeout_s and self.latest_status in ('navigating', 'goal_timeout_replan')

        msg = String()
        msg.data = f'status={self.latest_status};stuck={str(stuck).lower()};no_progress_s={elapsed:.1f}'
        self.heartbeat_pub.publish(msg)



def main() -> None:
    rclpy.init()
    node = StatusMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

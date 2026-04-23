#!/usr/bin/env python3
"""
Stall Guard — detects motor stalls and protects the drivetrain.

A stall is detected when a non-trivial velocity command is being sent to the
robot but the odometry reports near-zero motion for longer than stall_duration_s.

On detection the node:
  1. Immediately publishes a zero-velocity Twist to /cmd_vel.
  2. Cancels any active Nav2 NavigateToPose goal.
  3. Holds the robot stopped for recovery_duration_s to let the drivetrain
     cool and the battery voltage recover before Nav2 is allowed to resume.

This guards against the failure mode where the robot becomes physically stuck
(e.g. on furniture), the battery voltage sags under sustained stall current,
and the motors begin to squeal.
"""

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose
from nav_msgs.msg import Odometry


class StallGuard(Node):
    def __init__(self) -> None:
        super().__init__('stall_guard')

        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('odom_topic', '/odometry/filtered')
        # Minimum commanded speed to consider the robot "being driven"
        self.declare_parameter('cmd_threshold', 0.05)
        # Maximum measured speed to consider the robot "not actually moving"
        self.declare_parameter('motion_threshold', 0.03)
        # How long the stall condition must persist before triggering a response
        self.declare_parameter('stall_duration_s', 3.0)
        # How long to hold zero velocity after a stall is detected
        self.declare_parameter('recovery_duration_s', 3.0)
        self.declare_parameter('check_rate_hz', 10.0)

        cmd_vel_topic = self.get_parameter('cmd_vel_topic').get_parameter_value().string_value
        odom_topic = self.get_parameter('odom_topic').get_parameter_value().string_value
        self._cmd_threshold = self.get_parameter('cmd_threshold').get_parameter_value().double_value
        self._motion_threshold = self.get_parameter('motion_threshold').get_parameter_value().double_value
        self._stall_duration = Duration(
            seconds=self.get_parameter('stall_duration_s').get_parameter_value().double_value
        )
        self._recovery_duration = Duration(
            seconds=self.get_parameter('recovery_duration_s').get_parameter_value().double_value
        )
        check_rate = self.get_parameter('check_rate_hz').get_parameter_value().double_value

        self._cmd_vel = Twist()
        self._odom_vel = Twist()
        self._stall_start = None
        self._recovery_start = None
        self._in_recovery = False

        self.create_subscription(Twist, cmd_vel_topic, self._on_cmd_vel, 10)
        self.create_subscription(Odometry, odom_topic, self._on_odom, 20)

        self._cmd_pub = self.create_publisher(Twist, cmd_vel_topic, 10)

        # Only used for cancellation — we never send goals ourselves
        self._nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')

        self.create_timer(1.0 / max(1.0, check_rate), self._check)

        self.get_logger().info(
            f'Stall guard initialized '
            f'(stall_duration={self.get_parameter("stall_duration_s").get_parameter_value().double_value}s, '
            f'recovery_duration={self.get_parameter("recovery_duration_s").get_parameter_value().double_value}s)'
        )

    # ── Subscribers ──────────────────────────────────────────────────────────

    def _on_cmd_vel(self, msg: Twist) -> None:
        if not self._in_recovery:
            self._cmd_vel = msg

    def _on_odom(self, msg: Odometry) -> None:
        self._odom_vel = msg.twist.twist

    # ── Main check loop ──────────────────────────────────────────────────────

    def _check(self) -> None:
        now = self.get_clock().now()

        if self._in_recovery:
            # Keep publishing zeros until recovery window expires
            self._cmd_pub.publish(Twist())
            if (now - self._recovery_start) >= self._recovery_duration:
                self._in_recovery = False
                self._recovery_start = None
                self._cmd_vel = Twist()
                self.get_logger().info('Stall guard: recovery complete, resuming monitoring.')
            return

        if self._is_stalled():
            if self._stall_start is None:
                self._stall_start = now
            elif (now - self._stall_start) >= self._stall_duration:
                self._trigger_stall_response(now)
        else:
            self._stall_start = None

    def _is_stalled(self) -> bool:
        """True when a meaningful command is issued but the robot is not moving."""
        cmd_active = (
            abs(self._cmd_vel.linear.x) > self._cmd_threshold
            or abs(self._cmd_vel.angular.z) > self._cmd_threshold
        )
        not_moving = (
            abs(self._odom_vel.linear.x) < self._motion_threshold
            and abs(self._odom_vel.angular.z) < self._motion_threshold
        )
        return cmd_active and not_moving

    def _trigger_stall_response(self, now) -> None:
        self.get_logger().warn(
            f'STALL DETECTED — cmd=({self._cmd_vel.linear.x:.3f} m/s, '
            f'{self._cmd_vel.angular.z:.3f} rad/s) '
            f'odom=({self._odom_vel.linear.x:.3f} m/s, '
            f'{self._odom_vel.angular.z:.3f} rad/s). '
            f'Stopping robot and cancelling Nav2 goal.'
        )

        # Immediately stop the robot
        self._cmd_pub.publish(Twist())

        # Cancel any active Nav2 goal so it stops re-commanding
        if self._nav_client.server_is_ready():
            self._nav_client.cancel_all_goals_async()
        else:
            self.get_logger().warn('Stall guard: Nav2 action server not ready, could not cancel goal.')

        # Enter recovery hold
        self._in_recovery = True
        self._recovery_start = now
        self._stall_start = None


def main() -> None:
    rclpy.init()
    node = StallGuard()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool, String


class TeleopOverride(Node):
    def __init__(self) -> None:
        super().__init__('teleop_override')

        self.declare_parameter('toggle_topic', '/exploration/manual_toggle')
        self.declare_parameter('manual_mode_topic', '/exploration/manual_mode')
        self.declare_parameter('status_topic', '/exploration/status')
        self.declare_parameter('cmd_vel_auto_topic', '/cmd_vel_auto')
        self.declare_parameter('cmd_vel_manual_topic', '/cmd_vel_manual')
        self.declare_parameter('cmd_vel_output_topic', '/cmd_vel')
        self.declare_parameter('max_linear', 0.4)
        self.declare_parameter('max_angular', 1.2)

        self.toggle_topic = str(self.get_parameter('toggle_topic').value)
        self.manual_mode_topic = str(self.get_parameter('manual_mode_topic').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.cmd_vel_auto_topic = str(self.get_parameter('cmd_vel_auto_topic').value)
        self.cmd_vel_manual_topic = str(self.get_parameter('cmd_vel_manual_topic').value)
        self.cmd_vel_output_topic = str(self.get_parameter('cmd_vel_output_topic').value)
        self.max_linear = float(self.get_parameter('max_linear').value)
        self.max_angular = float(self.get_parameter('max_angular').value)

        self.manual_mode = False

        self.toggle_sub = self.create_subscription(Bool, self.toggle_topic, self.on_toggle, 10)
        self.auto_sub = self.create_subscription(Twist, self.cmd_vel_auto_topic, self.on_auto_cmd, 20)
        self.manual_sub = self.create_subscription(Twist, self.cmd_vel_manual_topic, self.on_manual_cmd, 20)

        self.cmd_pub = self.create_publisher(Twist, self.cmd_vel_output_topic, 20)
        self.mode_pub = self.create_publisher(Bool, self.manual_mode_topic, 10)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)

        self.publish_mode()
        self.get_logger().info('Teleop override initialized (toggle mode).')

    def on_toggle(self, msg: Bool) -> None:
        if not msg.data:
            return
        self.manual_mode = not self.manual_mode
        self.publish_mode()
        status = String()
        status.data = 'manual_override' if self.manual_mode else 'exploring'
        self.status_pub.publish(status)

        # Stop robot momentarily during mode handoff.
        self.cmd_pub.publish(Twist())

    def on_auto_cmd(self, msg: Twist) -> None:
        if not self.manual_mode:
            self.cmd_pub.publish(self.clamp(msg))

    def on_manual_cmd(self, msg: Twist) -> None:
        if self.manual_mode:
            self.cmd_pub.publish(self.clamp(msg))

    def clamp(self, msg: Twist) -> Twist:
        clamped = Twist()
        clamped.linear.x = max(-self.max_linear, min(self.max_linear, msg.linear.x))
        clamped.linear.y = max(-self.max_linear, min(self.max_linear, msg.linear.y))
        clamped.linear.z = max(-self.max_linear, min(self.max_linear, msg.linear.z))
        clamped.angular.x = max(-self.max_angular, min(self.max_angular, msg.angular.x))
        clamped.angular.y = max(-self.max_angular, min(self.max_angular, msg.angular.y))
        clamped.angular.z = max(-self.max_angular, min(self.max_angular, msg.angular.z))
        return clamped

    def publish_mode(self) -> None:
        msg = Bool()
        msg.data = self.manual_mode
        self.mode_pub.publish(msg)



def main() -> None:
    rclpy.init()
    node = TeleopOverride()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

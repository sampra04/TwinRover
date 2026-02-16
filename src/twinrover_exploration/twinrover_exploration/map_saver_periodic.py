#!/usr/bin/env python3
import os
from datetime import datetime

import rclpy
from rclpy.node import Node

from nav2_msgs.srv import SaveMap
from std_msgs.msg import String


class MapSaverPeriodic(Node):
    def __init__(self) -> None:
        super().__init__('map_saver_periodic')

        self.declare_parameter('autosave_interval_s', 120.0)
        self.declare_parameter('output_dir', 'maps')
        self.declare_parameter('status_topic', '/exploration/status')
        self.declare_parameter('save_map_timeout_s', 10.0)
        self.declare_parameter('map_url_prefix', 'warehouse_map')

        self.autosave_interval_s = float(self.get_parameter('autosave_interval_s').value)
        self.output_dir = str(self.get_parameter('output_dir').value)
        self.status_topic = str(self.get_parameter('status_topic').value)
        self.save_map_timeout_s = float(self.get_parameter('save_map_timeout_s').value)
        self.map_url_prefix = str(self.get_parameter('map_url_prefix').value)

        self.session_name = datetime.now().strftime('session_%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(self.output_dir, self.session_name)
        os.makedirs(self.session_dir, exist_ok=True)

        self.client = self.create_client(SaveMap, '/map_saver/save_map')
        self.status_sub = self.create_subscription(String, self.status_topic, self.on_status, 10)
        self.timer = self.create_timer(self.autosave_interval_s, self.on_autosave)

        self.get_logger().info(f'Map saver initialized. Session dir: {self.session_dir}')

    def on_status(self, msg: String) -> None:
        if msg.data == 'complete':
            self.get_logger().info('Exploration complete. Saving final map.')
            self.save_map('final')

    def on_autosave(self) -> None:
        self.save_map(datetime.now().strftime('autosave_%H%M%S'))

    def save_map(self, suffix: str) -> None:
        if not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('SaveMap service unavailable; skipping save.')
            return

        req = SaveMap.Request()
        req.map_topic = '/map'
        req.map_url = os.path.join(self.session_dir, f'{self.map_url_prefix}_{suffix}')
        req.image_format = 'pgm'
        req.map_mode = 'trinary'
        req.free_thresh = 0.25
        req.occupied_thresh = 0.65

        future = self.client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=self.save_map_timeout_s)

        if future.result() is not None and future.result().result:
            self.get_logger().info(f'Map saved: {req.map_url}.yaml/.pgm')
        else:
            self.get_logger().warn(f'Map save failed for: {req.map_url}')


def main() -> None:
    rclpy.init()
    node = MapSaverPeriodic()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.save_map('shutdown')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

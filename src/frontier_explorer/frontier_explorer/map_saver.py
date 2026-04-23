#!/usr/bin/env python3
import os
from datetime import datetime

import numpy as np
from PIL import Image

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid


class MapSaverNode(Node):
    def __init__(self) -> None:
        super().__init__('map_saver')

        self.declare_parameter('output_dir', '~/maps')

        output_dir = os.path.expanduser(str(self.get_parameter('output_dir').value))
        session_name = datetime.now().strftime('session_%Y%m%d_%H%M%S')
        self.session_dir = os.path.join(output_dir, session_name)
        os.makedirs(self.session_dir, exist_ok=True)

        self.map_data: OccupancyGrid | None = None
        self.create_subscription(OccupancyGrid, '/map', self._map_callback, 10)

        self.get_logger().info(f'Map saver ready. Will save PNG to: {self.session_dir} on shutdown.')

    def _map_callback(self, msg: OccupancyGrid) -> None:
        self.map_data = msg

    def _write_png(self) -> None:
        if self.map_data is None:
            self.get_logger().warn('No map received yet; nothing to save.')
            return

        width = self.map_data.info.width
        height = self.map_data.info.height
        arr = np.array(self.map_data.data, dtype=np.int8).reshape((height, width))

        img = np.full((height, width), 205, dtype=np.uint8)  # unknown → gray
        img[arr == 0] = 254   # free     → white
        img[arr > 0] = 0      # occupied → black

        # ROS maps have row 0 at the bottom; flip so north is up in the image
        img = np.flipud(img)

        path = os.path.join(self.session_dir, 'map_final.png')
        Image.fromarray(img).save(path)
        self.get_logger().info(f'Map saved: {path}')


def main() -> None:
    rclpy.init()
    node = MapSaverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node._write_png()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

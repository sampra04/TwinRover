#!/usr/bin/env python3
"""
TwinRover Object Fusion Node — Persistent Map
Fuses YOLOv8 camera detections with LiDAR scan to publish
object positions in the map frame as a persistent MarkerArray.

Objects are stored permanently once seen for CONFIRM_SECONDS.
A timer republishes all markers every second so they never expire.
"""

import rclpy
import rclpy.time
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import cv2
import numpy as np
import math
import time
from cv_bridge import CvBridge

from sensor_msgs.msg import Image, LaserScan
from visualization_msgs.msg import Marker, MarkerArray
from builtin_interfaces.msg import Duration

import tf2_ros
import tf2_geometry_msgs
from geometry_msgs.msg import PointStamped

from ultralytics import YOLO

# ── tuneable parameters ────────────────────────────────────────────────────────
MODEL_PATH       = "/home/twinrover/TwinRover/yolov8n_ncnn_model/"
IMGSZ            = 320
CONF             = 0.4
CAMERA_HFOV      = 78.0        # Logitech C925e horizontal FOV degrees
IMAGE_WIDTH      = 640
INFER_EVERY      = 5           # run YOLO every N frames
GRID_RESOLUTION  = 0.5         # meters — objects within this distance = same object
CONFIRM_SECONDS  = 3.0         # seconds an object must be seen before persisting
IMAGE_TOPIC      = "/image_raw"
SCAN_TOPIC       = "/scan"
# ──────────────────────────────────────────────────────────────────────────────

CLASS_COLORS = {
    "person":   (0.2, 0.6, 1.0, 0.9),
    "forklift": (1.0, 0.4, 0.0, 0.9),
    "pallet":   (0.2, 1.0, 0.4, 0.9),
}
DEFAULT_COLOR = (1.0, 0.8, 0.0, 0.9)


class ObjectFusionNode(Node):
    def __init__(self):
        super().__init__("object_fusion_node")
        self.get_logger().info("Object Fusion Node starting...")

        self.model  = YOLO(MODEL_PATH, task="detect")
        self.bridge = CvBridge()

        self.tf_buffer   = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        self.latest_scan = None
        self.frame_count = 0

        # Persistent object store
        # key:   (grid_x, grid_y, class_name)
        # value: {"first_seen": time, "confirmed": bool, "cyl": Marker, "lbl": Marker}
        self.persistent_objects = {}

        # Candidate buffer — tracks detections not yet confirmed
        # key: same as above, value: first seen timestamp
        self.candidates = {}

        # Marker ID counter (persistent across detections)
        self.next_marker_id = 0

        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            depth=1
        )

        self.image_sub = self.create_subscription(
            Image, IMAGE_TOPIC, self.image_callback, sensor_qos)
        self.scan_sub = self.create_subscription(
            LaserScan, SCAN_TOPIC, self.scan_callback, sensor_qos)

        self.marker_pub = self.create_publisher(MarkerArray, "/object_markers", 10)
        self.image_pub  = self.create_publisher(Image, "/object_detections_image", 10)

        # Republish all persistent markers every second
        self.persist_timer = self.create_timer(1.0, self.publish_persistent_markers)

        self.get_logger().info(f"Subscribed to {IMAGE_TOPIC} and {SCAN_TOPIC}")
        self.get_logger().info(
            f"Model: {MODEL_PATH}  FOV: {CAMERA_HFOV}deg  "
            f"Confirm after: {CONFIRM_SECONDS}s  Grid: {GRID_RESOLUTION}m")

    def scan_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def image_callback(self, msg: Image):
        self.frame_count += 1
        if self.frame_count % INFER_EVERY != 0:
            return
        if self.latest_scan is None:
            self.get_logger().warn("No scan yet, skipping", throttle_duration_sec=5.0)
            return

        frame     = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        results   = self.model(frame, imgsz=IMGSZ, conf=CONF, verbose=False)
        boxes     = results[0].boxes
        annotated = results[0].plot()

        now = time.time()

        for box in boxes:
            cls_id   = int(box.cls[0])
            cls_name = self.model.names[cls_id]
            conf_val = float(box.conf[0])
            x1, y1, x2, y2 = box.xyxy[0].tolist()

            cx_pixel  = (x1 + x2) / 2.0
            norm      = (cx_pixel / IMAGE_WIDTH) - 0.5
            angle_cam = -norm * math.radians(CAMERA_HFOV)

            depth = self.get_lidar_depth(angle_cam)
            if depth is None:
                continue

            pt_camera = PointStamped()
            pt_camera.header.stamp    = rclpy.time.Time().to_msg()
            pt_camera.header.frame_id = "camera_link"
            pt_camera.point.x = depth * math.cos(angle_cam)
            pt_camera.point.y = depth * math.sin(angle_cam)
            pt_camera.point.z = 0.0

            try:
                pt_map = self.tf_buffer.transform(
                    pt_camera, "map",
                    timeout=rclpy.duration.Duration(seconds=0.2))
            except Exception as e:
                self.get_logger().warn(f"TF failed: {e}", throttle_duration_sec=3.0)
                continue

            # Grid key for this detection
            gx  = round(pt_map.point.x / GRID_RESOLUTION)
            gy  = round(pt_map.point.y / GRID_RESOLUTION)
            key = (gx, gy, cls_name)

            # Already confirmed and stored — skip
            if key in self.persistent_objects:
                cv2.putText(annotated, f"{depth:.2f}m [saved]",
                            (int(x1), int(y1) - 10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
                continue

            # Track candidate
            if key not in self.candidates:
                self.candidates[key] = now
                self.get_logger().info(
                    f"Candidate {cls_name} at x={pt_map.point.x:.2f} "
                    f"y={pt_map.point.y:.2f} depth={depth:.2f}m — confirming...")

            elapsed = now - self.candidates[key]
            remaining = max(0.0, CONFIRM_SECONDS - elapsed)

            cv2.putText(annotated, f"{depth:.2f}m [{remaining:.1f}s]",
                        (int(x1), int(y1) - 10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 255), 1)

            # Confirm and persist after CONFIRM_SECONDS
            if elapsed >= CONFIRM_SECONDS:
                mid = self.next_marker_id
                self.next_marker_id += 2
                self.persistent_objects[key] = {
                    "cyl": self.make_cylinder(mid,     cls_name, pt_map),
                    "lbl": self.make_label   (mid + 1, cls_name, conf_val, pt_map),
                }
                del self.candidates[key]
                self.get_logger().info(
                    f"Confirmed {cls_name} at x={pt_map.point.x:.2f} "
                    f"y={pt_map.point.y:.2f} — added to persistent map "
                    f"({len(self.persistent_objects)} total)")

        self.image_pub.publish(
            self.bridge.cv2_to_imgmsg(annotated, encoding="bgr8"))

    def publish_persistent_markers(self):
        if not self.persistent_objects:
            return
        marker_array = MarkerArray()
        for entry in self.persistent_objects.values():
            marker_array.markers.append(entry["cyl"])
            marker_array.markers.append(entry["lbl"])
        self.marker_pub.publish(marker_array)

    def get_lidar_depth(self, angle_cam_rad: float):
        scan = self.latest_scan

        pt_in = PointStamped()
        pt_in.header.stamp    = rclpy.time.Time().to_msg()
        pt_in.header.frame_id = "camera_link"
        pt_in.point.x = math.cos(angle_cam_rad)
        pt_in.point.y = math.sin(angle_cam_rad)
        pt_in.point.z = 0.0

        try:
            pt_lidar = self.tf_buffer.transform(
                pt_in, "lidar_link",
                timeout=rclpy.duration.Duration(seconds=0.2))
        except Exception as e:
            self.get_logger().warn(f"Angle TF failed: {e}", throttle_duration_sec=3.0)
            return None

        bearing = math.atan2(pt_lidar.point.y, pt_lidar.point.x)

        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        idx = round((bearing - angle_min) / angle_inc)
        idx = max(0, min(idx, len(scan.ranges) - 1))

        window = 3
        lo = max(0, idx - window)
        hi = min(len(scan.ranges), idx + window + 1)
        valid = [r for r in scan.ranges[lo:hi]
                 if scan.range_min < r < scan.range_max]

        if not valid:
            return None

        return float(np.median(valid))

    def _lifetime(self):
        # 0 = never expire
        return Duration(sec=0, nanosec=0)

    def make_cylinder(self, mid: int, cls_name: str, pt_map: PointStamped) -> Marker:
        color = CLASS_COLORS.get(cls_name, DEFAULT_COLOR)
        m = Marker()
        m.header.stamp       = self.get_clock().now().to_msg()
        m.header.frame_id    = "map"
        m.ns                 = "object_detections"
        m.id                 = mid
        m.type               = Marker.CYLINDER
        m.action             = Marker.ADD
        m.pose.position.x    = pt_map.point.x
        m.pose.position.y    = pt_map.point.y
        m.pose.position.z    = 0.0
        m.pose.orientation.w = 1.0
        m.scale.x            = 0.4
        m.scale.y            = 0.4
        m.scale.z            = 0.5
        m.color.r            = color[0]
        m.color.g            = color[1]
        m.color.b            = color[2]
        m.color.a            = color[3]
        m.lifetime           = self._lifetime()
        return m

    def make_label(self, mid: int, cls_name: str, conf: float,
                   pt_map: PointStamped) -> Marker:
        m = Marker()
        m.header.stamp       = self.get_clock().now().to_msg()
        m.header.frame_id    = "map"
        m.ns                 = "object_labels"
        m.id                 = mid
        m.type               = Marker.TEXT_VIEW_FACING
        m.action             = Marker.ADD
        m.pose.position.x    = pt_map.point.x
        m.pose.position.y    = pt_map.point.y
        m.pose.position.z    = 0.6
        m.pose.orientation.w = 1.0
        m.scale.z            = 0.25
        m.color.r            = 1.0
        m.color.g            = 1.0
        m.color.b            = 1.0
        m.color.a            = 1.0
        m.text               = f"{cls_name} {conf:.2f}"
        m.lifetime           = self._lifetime()
        return m


def main(args=None):
    rclpy.init(args=args)
    node = ObjectFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
"""Save the latest image produced by the Autolabor-C1 driver on request."""

from __future__ import annotations

import os
from pathlib import Path
import threading
import time

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from agt_robot_interfaces.srv import CaptureCamera


def capture_filename(image: Image, sequence: int) -> str:
    """Return a collision-resistant filename derived from the image timestamp."""
    stamp = image.header.stamp
    return f'capture_{int(stamp.sec):010d}_{int(stamp.nanosec):09d}_{sequence:04d}.png'


def write_capture(image: Image, output_dir: Path, sequence: int, bridge: CvBridge) -> Path:
    """Convert one ROS image and atomically persist it as a PNG."""
    output_dir.mkdir(parents=True, exist_ok=True)
    destination = output_dir / capture_filename(image, sequence)
    temporary = destination.with_name(destination.stem + '.tmp.png')
    converted = bridge.imgmsg_to_cv2(image, desired_encoding='bgr8')
    if not cv2.imwrite(str(temporary), converted):
        raise RuntimeError(f'cv2.imwrite failed for {temporary}')
    os.replace(temporary, destination)
    return destination


class CameraCaptureService(Node):
    def __init__(self):
        super().__init__('camera_capture_service')
        self.declare_parameter('image_topic', '/cv_camera0/image_raw')
        self.declare_parameter('service_name', '/capability/camera/capture')
        self.declare_parameter('output_dir', '~/.ros/agt_navigation_debug/captures')
        self.declare_parameter('max_image_age_sec', 2.0)

        self._bridge = CvBridge()
        self._lock = threading.Lock()
        self._latest_image: Image | None = None
        self._latest_received_at = 0.0
        self._sequence = 0
        self._output_dir = Path(str(self.get_parameter('output_dir').value)).expanduser()

        self.create_subscription(
            Image,
            str(self.get_parameter('image_topic').value),
            self._on_image,
            qos_profile_sensor_data,
        )
        self.create_service(
            CaptureCamera,
            str(self.get_parameter('service_name').value),
            self._capture,
        )
        self.get_logger().info(
            f'camera capture service ready: {self.get_parameter("service_name").value}; '
            f'image topic={self.get_parameter("image_topic").value}; output={self._output_dir}'
        )

    def _on_image(self, image: Image) -> None:
        with self._lock:
            self._latest_image = image
            self._latest_received_at = time.monotonic()

    def _capture(self, _request, response: CaptureCamera.Response) -> CaptureCamera.Response:
        with self._lock:
            image = self._latest_image
            received_at = self._latest_received_at
            self._sequence += 1
            sequence = self._sequence

        max_age = float(self.get_parameter('max_image_age_sec').value)
        age = time.monotonic() - received_at
        if image is None or age > max_age:
            response.success = False
            response.message = (
                'no fresh C1 image available'
                if image is None else f'C1 image is stale ({age:.2f}s > {max_age:.2f}s)'
            )
            return response

        try:
            saved = write_capture(image, self._output_dir, sequence, self._bridge)
        except Exception as exc:
            response.success = False
            response.message = f'capture failed: {exc}'
            self.get_logger().error(response.message)
            return response

        response.success = True
        response.image_path = str(saved)
        response.message = 'captured current Autolabor-C1 frame'
        self.get_logger().info(f'captured image: {saved}')
        return response


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CameraCaptureService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

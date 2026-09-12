from pathlib import Path

import numpy as np
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

from agt_capability_camera.camera_capture_service import capture_filename, write_capture


def _image() -> Image:
    image = CvBridge().cv2_to_imgmsg(np.zeros((4, 6, 3), dtype=np.uint8), encoding='bgr8')
    image.header.stamp.sec = 123
    image.header.stamp.nanosec = 456
    return image


def test_capture_filename_uses_image_stamp_and_sequence():
    assert capture_filename(_image(), 7) == 'capture_0000000123_000000456_0007.png'


def test_write_capture_persists_a_png(tmp_path: Path):
    saved = write_capture(_image(), tmp_path / 'captures', 1, CvBridge())
    assert saved.is_file()
    assert saved.read_bytes().startswith(b'\x89PNG')

from pathlib import Path
from unittest.mock import Mock

import cv2
import numpy as np
import pytest
import requests

import img_send
import master_node
from image_detect import ImageDetector


def test_image_position_keeps_decimal_and_negative_coordinates():
    assert img_send.image_position("/tmp/img/1750000000_1.25_-2.5_0.0.jpg") == [1.25, -2.5, 0.0]


def test_camera_callback_only_keeps_latest_frame_and_retains_bounded_files(tmp_path, monkeypatch):
    converter = Mock(return_value=np.zeros((8, 8, 3), dtype=np.uint8))
    monkeypatch.setattr(master_node, "bridge", Mock(imgmsg_to_cv2=converter))
    monkeypatch.setattr(master_node, "IMG_SAVE_DIR", str(tmp_path))
    monkeypatch.setattr(master_node, "MAX_SAVED_IMAGES", 3)
    monkeypatch.setattr(master_node, "current_pos", [1.25, -2.5, 0.0])
    for index in range(100):
        master_node.image_callback(index)
    converter.assert_not_called()
    assert list(tmp_path.iterdir()) == []
    master_node.save_latest_image()
    assert converter.call_args.args[0] == 99
    first = next(tmp_path.glob("*.jpg"))
    assert img_send.image_position(first) == [1.25, -2.5, 0.0]
    assert cv2.imread(str(first)).shape == (8, 8, 3)
    for index in range(5):
        master_node.image_callback(index)
        master_node.save_latest_image()
    assert len(list(tmp_path.glob("*.jpg"))) == 3


def test_upload_failure_is_not_reported_as_success(tmp_path):
    image = tmp_path / "image.jpg"
    image.write_bytes(b"jpeg")
    session = Mock()
    session.post.side_effect = requests.Timeout()
    assert img_send.img_send(image, [0, 0, 0], session) is None
    assert session.post.call_args.kwargs["timeout"] == (5, 45)


def test_failed_http_response_is_not_treated_as_no_detection(tmp_path):
    image = tmp_path / "image.jpg"
    image.write_bytes(b"jpeg")
    session = Mock()
    session.post.return_value.raise_for_status.side_effect = requests.HTTPError()
    assert img_send.img_send(image, [0, 0, 0], session) is None


def test_throttled_detection_does_not_sleep(monkeypatch):
    detector = ImageDetector.__new__(ImageDetector)
    detector.last_time = 10
    import image_detect
    monkeypatch.setattr(image_detect.time, "monotonic", lambda: 10.5)
    sleep = Mock(side_effect=AssertionError("callback must not sleep"))
    monkeypatch.setattr(image_detect.time, "sleep", sleep)
    detector.image_callback(object())
    sleep.assert_not_called()

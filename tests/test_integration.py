import json
import os
from pathlib import Path
import queue
import socket
import subprocess
import sys
import threading
import time

import actionlib
import cv2
import numpy as np
import pytest
import rospy
import yaml
from geometry_msgs.msg import Pose, Twist
from move_base_msgs.msg import MoveBaseAction, MoveBaseResult
from nav_msgs.msg import OccupancyGrid
from nav_msgs.srv import GetMap, GetMapResponse
from Aft_g1.msg import NavControl
from Aft_g1.srv import MasterNode, MasterNodeRequest


SCRIPTS = Path(__file__).resolve().parents[1] / "src" / "Aft_g1" / "src"


@pytest.fixture
def processes(tmp_path):
    children = []
    logs = []

    def launch(command, name):
        output = (tmp_path / (name + ".log")).open("w+")
        process = subprocess.Popen(command, stdout=output, stderr=subprocess.STDOUT)
        children.append(process)
        logs.append(output)
        return process

    yield launch
    for process in reversed(children):
        if process.poll() is None:
            process.terminate()
            try:
                process.wait(timeout=6)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=2)
    for output in logs:
        output.close()


def wait_until(predicate, timeout=5):
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        if predicate():
            return
        time.sleep(0.02)
    pytest.fail("Timed out waiting for ROS state")


def test_real_nodes_patrol_cancel_and_save_map(ros_master, processes, tmp_path):
    goals = queue.Queue()
    hold = threading.Event()
    preempted = threading.Event()
    arrivals = []

    def execute(goal):
        goals.put(goal)
        while hold.is_set() and not rospy.is_shutdown():
            if server.is_preempt_requested():
                server.set_preempted(MoveBaseResult())
                preempted.set()
                return
            time.sleep(0.01)
        if server.is_active():
            server.set_succeeded(MoveBaseResult())

    server = actionlib.SimpleActionServer("move_base", MoveBaseAction, execute_cb=execute, auto_start=False)
    server.start()
    listener = rospy.Subscriber("/nav/arrive", NavControl, arrivals.append)
    image = tmp_path / "test_map.pgm"
    cv2.imwrite(str(image), np.full((20, 20), 255, dtype=np.uint8))
    (tmp_path / "test_map.yaml").write_text(yaml.safe_dump({
        "image": str(image), "resolution": 0.1, "origin": [-1.0, -2.0, 0.0],
        "negate": 0, "occupied_thresh": 0.65, "free_thresh": 0.196,
    }))
    processes([sys.executable, str(SCRIPTS / "nav_node.py"), "_goal_timeout:=5"], "nav_node")
    processes([sys.executable, str(SCRIPTS / "nav_control_node.py"), f"_maps_dir:={tmp_path}"], "nav_control")
    processes([sys.executable, str(SCRIPTS / "master_node.py"), f"_maps_dir:={tmp_path}", f"_image_dir:={tmp_path / 'images'}"], "master")
    rospy.wait_for_service("/master_node", timeout=10)
    master = rospy.ServiceProxy("/master_node", MasterNode)
    try:
        # Service registration precedes subscriber handshakes in separate nodes.
        wait_until(lambda: len(rospy.get_published_topics()) > 4)
        request = MasterNodeRequest(type=30)
        request.navigation_ctrl_msg.name_list = ["test_map"]
        assert master(request).code == 0
        rospy.wait_for_message("/map", OccupancyGrid, timeout=8)
        control = MasterNodeRequest(type=32)
        first, second = Pose(), Pose()
        first.position.x, second.position.x = 1.25, -2.5
        first.orientation.w = second.orientation.w = 1
        control.navigation_ctrl_msg.pose_list = [first, second]
        control.navigation_ctrl_msg.name_list = ["first", "second"]
        control.navigation_ctrl_msg.loop = 0
        assert master(control).code == 0
        assert goals.get(timeout=5).target_pose.pose.position.x == 1.25
        assert goals.get(timeout=5).target_pose.pose.position.x == -2.5
        wait_until(lambda: len(arrivals) == 2)
        with pytest.raises(queue.Empty):
            goals.get(timeout=0.2)

        hold.set()
        assert master(control).code == 0
        goals.get(timeout=5)
        assert master(MasterNodeRequest(type=34)).code == 0
        assert preempted.wait(5)
        with pytest.raises(queue.Empty):
            goals.get(timeout=0.2)
        assert len(arrivals) == 2
        assert master(MasterNodeRequest(type=31)).code == 0
        with pytest.raises(queue.Empty):
            goals.get(timeout=0.2)

        assert master(MasterNodeRequest(type=20)).code == 0
        save = MasterNodeRequest(type=23)
        save.navigation_ctrl_msg.name_list = ["saved_map"]
        assert master(save).code == 0
        assert (tmp_path / "saved_map.pgm").is_file()
        assert (tmp_path / "saved_map.yaml").is_file()
        assert master(MasterNodeRequest(type=21)).code == 0
    finally:
        hold.clear()
        listener.unregister()


def test_django_rosbridge_disconnect_and_reconnect(ros_master, processes, tmp_path):
    backend = Path(__file__).resolve().parents[2] / "BackEnd"
    backend_python = Path(sys.prefix).parent / "ros-buaa-backend" / "bin" / "python"
    if not backend_python.exists():
        pytest.skip("Create the ros-buaa-backend environment for this cross-repository test")
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    config = tmp_path / "backend-test.yaml"
    config.write_text(yaml.safe_dump({"DjangoSecretKey": "integration-test-only", "ROSHOST": "127.0.0.1", "ROSPORT": port, "ROS_SERVICE_TIMEOUT": 3}))
    calls = []
    from Aft_g1.srv import MasterNodeResponse
    service = rospy.Service("/master_node", MasterNode, lambda request: (calls.append(request), MasterNodeResponse(code=0))[1])
    processes(["roslaunch", "rosbridge_server", "rosbridge_websocket.launch", "address:=127.0.0.1", f"port:={port}"], "rosbridge")
    def port_ready():
        try:
            with socket.create_connection(("127.0.0.1", port), timeout=0.1):
                return True
        except OSError:
            return False
    wait_until(port_ready, timeout=10)
    try:
        result = subprocess.run(
            [str(backend_python), str(Path(__file__).with_name("backend_bridge_probe.py"))],
            cwd=backend,
            env={**os.environ, "ROS_BUAA_CONFIG": str(config), "PYTHONNOUSERSITE": "1", "PYTHONPATH": str(backend)},
            capture_output=True, text=True, timeout=30,
        )
        assert result.returncode == 0, result.stdout + result.stderr
        assert len(calls) == 2
        assert all(request.type == 12 and request.keyboard_ctrl_msg.direction == 1 for request in calls)
    finally:
        service.shutdown()

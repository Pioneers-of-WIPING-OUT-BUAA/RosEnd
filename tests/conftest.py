import os
import signal
import socket
import subprocess
import time

import pytest
import rosgraph
import rospy


@pytest.fixture(scope="session")
def ros_master(tmp_path_factory):
    with socket.socket() as sock:
        sock.bind(("127.0.0.1", 0))
        port = sock.getsockname()[1]
    directory = tmp_path_factory.mktemp("ros")
    previous = {key: os.environ.get(key) for key in ("ROS_MASTER_URI", "ROS_HOSTNAME", "ROS_LOG_DIR")}
    os.environ.update(ROS_MASTER_URI=f"http://127.0.0.1:{port}", ROS_HOSTNAME="127.0.0.1", ROS_LOG_DIR=str(directory))
    with (directory / "roscore.log").open("w+") as output:
        process = subprocess.Popen(["roscore", "-p", str(port)], stdout=output, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            deadline = time.monotonic() + 20
            while time.monotonic() < deadline:
                try:
                    rosgraph.Master("/test_environment").getPid()
                    break
                except (OSError, rosgraph.MasterException):
                    time.sleep(0.1)
            else:
                output.seek(0)
                pytest.fail("roscore did not start:\n" + output.read())
            rospy.init_node("ros_buaa_tests", anonymous=True, disable_signals=True)
            yield
        finally:
            rospy.signal_shutdown("Tests finished")
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=8)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=3)
            for key, value in previous.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value

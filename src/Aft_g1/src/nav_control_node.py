#!/usr/bin/env python
# -*- coding: utf-8 -*-

import subprocess
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from std_msgs.msg import String
from Aft_g1.msg import NavControl, ErrorMsg
from map_files import map_path


class NavControlNode:
    def __init__(self):
        rospy.init_node("NavControl", anonymous=True)
        self.control = NavControl()
        self.status = 0
        self.map_process = None
        self.maps_dir = rospy.get_param("~maps_dir", "/home/robot/maps")
        self.ctrl_pub_stop = rospy.Publisher("/nav/stop", String, queue_size=10)
        self.ctrl_pub_nav = rospy.Publisher("/nav/nav", NavControl, queue_size=10)
        self.ctrl_pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.error_pub = rospy.Publisher("/error", ErrorMsg, queue_size=10)
        self.subscriber = rospy.Subscriber("/nav", NavControl, self.chatter_callback, queue_size=10)
        rospy.on_shutdown(self.shutdown)

    def chatter_callback(self, msg):
        self.control = msg
        try:
            if msg.op == 0:
                self.handle_map_loading()
            elif msg.op == 1:
                self.handle_stop_navigation()
            elif msg.op == 2:
                self.handle_waypoint_patrol()
            elif msg.op == 3:
                self.handle_emergency_stop()
            else:
                self.error_pub.publish(ErrorMsg(type=0, message="不支持的导航指令"))
        except (OSError, ValueError, subprocess.SubprocessError, rospy.ROSException) as exc:
            self.status = 0
            self.handle_emergency_stop()
            self.error_pub.publish(ErrorMsg(type=0, message="导航初始化失败"))
            rospy.logerr("Navigation control failed: %s", exc)

    def _stop_map_server(self):
        if self.map_process is not None:
            self.map_process.terminate()
            try:
                self.map_process.wait(timeout=2)
            except subprocess.TimeoutExpired:
                self.map_process.kill()
                self.map_process.wait(timeout=2)
            self.map_process = None

    def handle_map_loading(self):
        path = map_path(self.control.map_name, self.maps_dir, ".yaml")
        if not path.is_file():
            raise ValueError("Map file does not exist")
        self.status = 0
        self.handle_emergency_stop()
        self._stop_map_server()
        ready = threading.Event()
        started = rospy.Time.now()
        listener = rospy.Subscriber(
            "/map", OccupancyGrid,
            lambda message: ready.set() if message.header.stamp >= started else None,
            queue_size=1,
        )
        try:
            self.map_process = subprocess.Popen([
                "rosrun", "map_server", "map_server", str(path), "__name:=map_server",
            ])
            if not ready.wait(4) or self.map_process.poll() is not None:
                self._stop_map_server()
                raise rospy.ROSException("Map server did not become ready")
            self.status = 1
        finally:
            listener.unregister()

    def handle_stop_navigation(self):
        self.status = 0
        self.handle_emergency_stop()

    def handle_waypoint_patrol(self):
        if self.status != 1 or not self.control.poses:
            self.error_pub.publish(ErrorMsg(type=0, message="请先加载地图并选择航点"))
            return
        self.ctrl_pub_nav.publish(self.control)

    def handle_emergency_stop(self):
        self.ctrl_pub_stop.publish(String())
        self.ctrl_pub_vel.publish(Twist())

    def shutdown(self):
        self.handle_emergency_stop()
        self._stop_map_server()

    def handle_operation(self):
        rospy.spin()


if __name__ == "__main__":
    try:
        NavControlNode().handle_operation()
    except rospy.ROSInterruptException:
        pass

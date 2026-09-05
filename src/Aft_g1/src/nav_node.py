#!/usr/bin/env python
# -*- coding: utf-8 -*-

from copy import deepcopy
import math
import threading
import time

import rospy
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from std_msgs.msg import String
from Aft_g1.msg import NavControl, ErrorMsg


class NavController:
    def __init__(self):
        rospy.init_node("nav_py")
        self.lock = threading.RLock()
        self.wake = threading.Event()
        self.generation = 0
        self.status = 0
        self.control = NavControl()
        self.goal_timeout = float(rospy.get_param("~goal_timeout", 120))
        self.ac = SimpleActionClient("move_base", MoveBaseAction)
        self.arrive_pub = rospy.Publisher("/nav/arrive", NavControl, queue_size=10)
        self.error_pub = rospy.Publisher("/error", ErrorMsg, queue_size=10)
        self.subscribers = [
            rospy.Subscriber("/nav/stop", String, self.sub_callback),
            rospy.Subscriber("/nav/nav", NavControl, self.nav_callback),
        ]
        rospy.on_shutdown(self.shutdown)

    def nav_callback(self, msg):
        if not msg.poses or msg.loop not in (0, 1) or len(msg.names) not in (0, len(msg.poses)):
            self.error_pub.publish(ErrorMsg(type=0, message="导航航点或循环模式无效"))
            return
        for pose in msg.poses:
            if not all(math.isfinite(value) for value in (
                pose.position.x, pose.position.y, pose.position.z,
                pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w,
            )):
                self.error_pub.publish(ErrorMsg(type=0, message="导航坐标包含无效数值"))
                return
        with self.lock:
            if self.status:
                self.ac.cancel_goal()
            self.generation += 1
            self.control = deepcopy(msg)
            self.status = 1
            self.wake.set()

    def sub_callback(self, msg):
        with self.lock:
            self.generation += 1
            self.status = 0
            self.ac.cancel_goal()
            self.wake.set()

    def _current(self, generation):
        with self.lock:
            return self.status == 1 and self.generation == generation and not rospy.is_shutdown()

    def _finish(self, generation, error=None):
        with self.lock:
            if self.generation != generation:
                return
            self.status = 0
            if error:
                self.ac.cancel_goal()
                self.error_pub.publish(ErrorMsg(type=0, message=error))

    def _execute_path(self, generation, control):
        if not self.ac.wait_for_server(rospy.Duration(2)):
            self._finish(generation, "导航服务不可用")
            return
        while self._current(generation):
            for index, pose in enumerate(control.poses):
                goal = MoveBaseGoal()
                goal.target_pose.header.frame_id = "map"
                goal.target_pose.header.stamp = rospy.Time.now()
                goal.target_pose.pose = deepcopy(pose)
                orientation = goal.target_pose.pose.orientation
                norm = math.sqrt(sum(value * value for value in (orientation.x, orientation.y, orientation.z, orientation.w)))
                if norm == 0:
                    orientation.w = 1
                else:
                    orientation.x /= norm
                    orientation.y /= norm
                    orientation.z /= norm
                    orientation.w /= norm
                with self.lock:
                    if not self._current(generation):
                        return
                    self.ac.send_goal(goal)
                deadline = time.monotonic() + self.goal_timeout
                while self._current(generation):
                    result = self.ac.get_state()
                    if result == GoalStatus.SUCCEEDED:
                        break
                    if result in (GoalStatus.ABORTED, GoalStatus.REJECTED, GoalStatus.PREEMPTED, GoalStatus.RECALLED, GoalStatus.LOST):
                        self._finish(generation, "导航目标失败或被取消")
                        return
                    if time.monotonic() >= deadline:
                        self._finish(generation, "导航目标超时")
                        return
                    self.wake.wait(0.05)
                    self.wake.clear()
                with self.lock:
                    if not self._current(generation):
                        return
                    self.arrive_pub.publish(NavControl(
                        op=2, poses=[pose],
                        names=[control.names[index]] if control.names else [],
                        loop=control.loop,
                    ))
            if control.loop == 0:
                self._finish(generation)
                return

    def shutdown(self):
        self.sub_callback(String())

    def run(self):
        while not rospy.is_shutdown():
            with self.lock:
                task = (self.generation, deepcopy(self.control)) if self.status else None
            if task:
                self._execute_path(*task)
            else:
                self.wake.wait(0.1)
                self.wake.clear()


if __name__ == "__main__":
    try:
        NavController().run()
    except rospy.ROSInterruptException:
        pass

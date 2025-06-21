#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import requests
import json
import os
import time

from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from actionlib.simple_action_client import SimpleActionClient
from actionlib_msgs.msg import GoalStatus, GoalStatusArray
from move_base_msgs.msg import MoveBaseActionResult
from std_msgs.msg import String
from Aft_g1.msg import NavControl


class NavController:
    def __init__(self):
        rospy.init_node('nav_py')

        # 状态变量
        self.hascancel = 0
        self.run_flag = 1
        self.status = 0
        self.loop = 0
        self.control = NavControl()
        self.ac = SimpleActionClient("move_base", MoveBaseAction)
        self.origin = rospy.get_param('/map_origin', [0.0, 0.0, 0.0])

        rospy.Subscriber("/nav/stop", String, self.sub_callback)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        rospy.Subscriber("/move_base/result", MoveBaseActionResult, self.result_callback)
        rospy.Subscriber("/nav/nav", NavControl, self.nav_callback)

        self.arrive_pub = rospy.Publisher("/nav/arrive", NavControl, queue_size=10)

        # 等待动作服务器
        while not self.ac.wait_for_server(rospy.Duration(5.0)):
            if rospy.is_shutdown():
                return
            rospy.loginfo("Waiting for move_base server connection...")

    def nav_callback(self, msg):
        self.status = 1
        self.hascancel = 0
        self.control = msg
        self.loop = msg.loop
        rospy.loginfo(f"begin navigation with loop: {self.loop}, poses: {len(self.control.poses)}")

    def status_callback(self, status_array):
        for status in status_array.status_list:
            if status.status == GoalStatus.ACTIVE:
                rospy.loginfo("Navigation in progress...")
            elif status.status == GoalStatus.ABORTED:
                rospy.loginfo("Navigation aborted!")
                self.run_flag = 0
                if not self.hascancel:
                    self.ac.cancel_goal()
                    self.hascancel = 1

    def result_callback(self, result):
        if result.status.status == GoalStatus.ABORTED:
            rospy.loginfo("Path planning failed!")
        elif result.status.status == GoalStatus.SUCCEEDED:
            rospy.loginfo("Navigation completed successfully!")
            self.run_flag = 0
        elif result.status.status == GoalStatus.PREEMPTED:
            rospy.loginfo("Navigation interrupted!")

    def sub_callback(self, msg):
        rospy.loginfo("Received stop signal")
        self.ac.cancel_goal()
        self.hascancel = 1

    def run(self):
        while not rospy.is_shutdown():
            rospy.sleep(0.1)
            if self.status == 0:
                continue

            poses = self.control.poses
            while self.loop > 0:
                rospy.loginfo("Starting loop navigation")
                for idx, pose in enumerate(poses):
                    if rospy.is_shutdown():
                        return

                    goal = MoveBaseGoal()
                    goal.target_pose.header.frame_id = "map"
                    goal.target_pose.header.stamp = rospy.Time.now()
                    goal.target_pose.pose.position.x = pose.position.x + self.origin[0]
                    goal.target_pose.pose.position.y = pose.position.y + self.origin[1]
                    goal.target_pose.pose.position.z = pose.position.z + self.origin[2]
                    goal.target_pose.pose.orientation = pose.orientation

                    self.ac.send_goal(goal)
                    rospy.loginfo(f"Sending {idx}th navigation point: {goal.target_pose.pose.position}")

                    while True:
                        if self.ac.get_state() == GoalStatus.SUCCEEDED:
                            break
                        if self.hascancel:
                            break
                        rospy.sleep(0.1)

                    if self.hascancel:
                        break

                    if self.ac.get_state() == GoalStatus.SUCCEEDED:
                        rospy.loginfo(f"Reached {idx}th waypoint")
                    else:
                        rospy.loginfo(f"Failed to reach {idx}th waypoint")

                self.loop -= 1
                self.arrive_pub.publish(NavControl())

                if self.hascancel:
                    self.loop = 0
                    break

            # Reset state
            if self.hascancel:
                self.hascancel = 0
                self.status = 0
            self.status = 0

if __name__ == "__main__":
    try:
        controller = NavController()
        controller.run()
    except rospy.ROSInterruptException:
        pass
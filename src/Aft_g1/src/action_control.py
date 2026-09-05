#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import subprocess
import time
import signal
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState
from Aft_g1.msg import NavControl
import threading
import std_msgs.msg

# 机械臂动作类型常量
ARM_OUT = 8      # 出机械臂
ARM_IN = 9       # 收机械臂
ARM_UP = 10       # 机械臂上升
ARM_DOWN = 11     # 机械臂下降
GRIP_CLOSE = 12   # 机械臂抓紧
GRIP_OPEN = 13    # 机械臂松开
ARM_STOP = 14     # 机械臂停止

class ActionControlNode:
    def __init__(self):
        rospy.init_node('act_ctrl')

        self.is_grab = False
        self.grab_cancel = threading.Event()
        self.direction = 0
        self.speed = 0.0
        self.command = ""
        self.now_arm_height = 0.0
        self.now_arm_distance = 0.0
        self.control = NavControl()
        self.current_lift = 0.5      # 记录当前升降高度
        self.current_gripper = 0.1   # 记录当前夹爪开合

        self.ctrl_pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.ctrl_pub_mani = rospy.Publisher("/wpb_home/mani_ctrl", JointState, queue_size=30)
        self.behaviors_pub = rospy.Publisher("/wpb_home/behaviors", std_msgs.msg.String, queue_size=30)
        self.grab_done = False
        self.grab_event = threading.Event()

        rospy.Subscriber("/act", NavControl, self.act_callback)
        rospy.Subscriber("/wpb_home/grab_result", std_msgs.msg.String, self.grab_result_callback)

        rospy.loginfo("robot control node initialized")

    def act_callback(self, msg):
        op = msg.op
        self.control = msg

        if msg.direction in (0, ARM_STOP):
            self.grab_cancel.set()
            self.direction = msg.direction
            self.speed = 0.0
            if msg.direction == ARM_STOP:
                self.control_arm(ARM_STOP)
            else:
                self.handle_manual_control()
            return

        if not self.is_grab:
            self.direction = msg.direction
            # 运动控制
            if op == 1:
                self.speed = msg.speed
                self.handle_manual_control()
            # 兼容原有抓取流程
            elif op == 2:
                if self.direction == 7:
                    self.is_grab = True
                    self.grab_cancel.clear()
                    threading.Thread(target=self.grab_object, daemon=True).start()
                else:
                    self.control_arm(self.direction)

    def handle_manual_control(self):
        rospy.loginfo("[manual control] direction: %d, speed: %f", self.direction, self.speed)
        vel_cmd = Twist()
        vel_cmd.linear.x = 0
        vel_cmd.linear.y = 0
        vel_cmd.linear.z = 0
        vel_cmd.angular.x = 0
        vel_cmd.angular.y = 0
        vel_cmd.angular.z = 0
        
        if self.direction == 0:
            pass
        elif self.direction == 1:
            vel_cmd.linear.x = self.speed
        elif self.direction == 2:
            vel_cmd.linear.x = -self.speed
        elif self.direction == 3:
            vel_cmd.linear.y = self.speed
        elif self.direction == 4:
            vel_cmd.linear.y = -self.speed
        elif self.direction == 5:
            vel_cmd.angular.z = self.speed
        elif self.direction == 6:
            vel_cmd.angular.z = -self.speed

        self.ctrl_pub_vel.publish(vel_cmd)

    def grab_result_callback(self, msg):
        """
        Listen for grab result callback. If 'done' or 'success' is received, grabbing is successful, otherwise failed.
        """
        rospy.logwarn("[GrabResultCB] %s", msg.data)
        if "done" in msg.data or "success" in msg.data:
            self.grab_done = True
            self.grab_event.set()
        elif "failed" in msg.data:
            self.grab_done = False
            self.grab_event.set()

    def grab_object(self):
        """
        Directly control the manipulator to complete the grab action:
        1. Publish JointState to /wpb_home/mani_ctrl to control lift and gripper.
        2. Wait for the action to complete (e.g., 3 seconds).
        3. Actively publish the grab result to /wpb_home/grab_result.
        """
        self.is_grab = True
        try:
            js = JointState()
            # msg.position = [升降高度, 夹爪开合距离]
            # msg.velocity = [升降速度, 夹爪开合速度]
            js.name = ["lift", "gripper"]
            js.position = [1.0, 0.0]  # 1.0m up, 0.0 closed
            js.velocity = [0.5, 5.0]  # lift speed 0.5m/s, gripper 5deg/s
            self.ctrl_pub_mani.publish(js)
            rospy.loginfo("Published grab action command to /wpb_home/mani_ctrl")
            if self.grab_cancel.wait(3):
                return

            result_pub = rospy.Publisher("/wpb_home/grab_result", std_msgs.msg.String, queue_size=1)
            msg = std_msgs.msg.String()
            msg.data = "done"
            result_pub.publish(msg)
            rospy.loginfo("Grab action completed, result published.")
        except Exception as e:
            rospy.logerr("Error occurred during grabbing: %s", str(e))
        finally:
            self.is_grab = False

    def control_arm(self, action_type):
        """
        Control manipulator actions:
        1. Arm out 2. Arm in 3. Up 4. Down 5. Grip close 6. Grip open 7. Stop
        """
        js = JointState()
        js.name = ["lift", "gripper"]
        current_lift = self.current_lift
        current_gripper = self.current_gripper
        js.velocity = [0.5, 5.0]

        if action_type == ARM_OUT:
            js.position = [0.5, 0.1]
            self.current_lift, self.current_gripper = 0.5, 0.1
            rospy.loginfo("Arm out (ready)")
        elif action_type == ARM_IN:
            js.position = [0.0, 0.1]
            self.current_lift, self.current_gripper = 0.0, 0.1
            rospy.loginfo("Arm in")
        elif action_type == ARM_UP:
            js.position = [1.0, current_gripper]
            self.current_lift = 1.0
            rospy.loginfo("Arm up")
        elif action_type == ARM_DOWN:
            js.position = [0.5, current_gripper]
            self.current_lift = 0.5
            rospy.loginfo("Arm down")
        elif action_type == GRIP_CLOSE:
            js.position = [current_lift, 0.0]
            js.velocity = [0.0, 5.0]
            self.current_gripper = 0.0
            rospy.loginfo("Grip close (gripper only)")
        elif action_type == GRIP_OPEN:
            js.position = [current_lift, 0.1]
            js.velocity = [0.0, 5.0]
            self.current_gripper = 0.1
            rospy.loginfo("Grip open (gripper only)")
        elif action_type == ARM_STOP:
            js.position = [current_lift, current_gripper]
            js.velocity = [0.0, 0.0]
            rospy.loginfo("Arm stop (force current position as target, publish multiple times)")
            self.ctrl_pub_mani.publish(js)
            return
        else:
            rospy.logwarn("Unknown arm action type: %s", action_type)
            return

        self.ctrl_pub_mani.publish(js)
        rospy.loginfo(f"Arm action published: {action_type}, position={js.position}, velocity={js.velocity}")

    def run(self):
        rospy.loginfo("robot control node start running")
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()

if __name__ == '__main__':
    try:
        act_ctrl = ActionControlNode()
        act_ctrl.run()
    except rospy.ROSInterruptException:
        rospy.logerr("[ERROR] robot control error!")

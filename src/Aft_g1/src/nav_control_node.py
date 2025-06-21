#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import subprocess
from std_msgs.msg import String
from geometry_msgs.msg import Twist, Pose, PoseStamped
from sensor_msgs.msg import JointState
from Aft_g1.msg import NavControl
import time

class NavControlNode:
    def __init__(self):
        rospy.init_node('NavControl', anonymous=True)

        self.status = 0
        self.receive = 0
        self.isbreak = 0
        self.cKey = 0
        self.op = 0
        self.flag_initial = 0
        self.direction = 0
        self.speed = 0.0
        self.command = ""
        self.has_arrive = 0
        self.now_arm_height = 0.0
        self.now_arm_distance = 0.0
        self.name = ""
        self.control = NavControl()
        self.current_target_index = 0  # 当前目标点索引
        self.is_navigating = False     # 是否正在导航中

        # 发布器
        self.ctrl_pub_stop = rospy.Publisher("/nav/stop", String, queue_size=10)
        self.ctrl_pub_nav = rospy.Publisher("/nav/nav", NavControl, queue_size=10)
        self.ctrl_pub_start = rospy.Publisher("/nav/start", String, queue_size=10)
        self.ctrl_pub_vel = rospy.Publisher("/cmd_vel", Twist, queue_size=10)
        self.ctrl_pub_mani = rospy.Publisher("/wpb_home/mani_ctrl", JointState, queue_size=30)
        self.ctrl_pub_initial = rospy.Publisher("/pub_initial", String, queue_size=30)
        self.move_base_pub = rospy.Publisher("/move_base_simple/goal", PoseStamped, queue_size=10)

        # 订阅器
        rospy.Subscriber("/nav", NavControl, self.chatter_callback)
        rospy.Subscriber("/nav/arrive", NavControl, self.arrive_callback)

        self.image_process = None
        rospy.loginfo("NavControlNode initialized!")

    def chatter_callback(self, msg):
        self.receive = 1
        self.control = msg
        self.cKey = msg.op
        self.direction = msg.direction
        self.speed = msg.speed
        self.command = msg.command
        self.name = msg.map_name
        rospy.loginfo(f"Received command: op={self.cKey}, direction={self.direction}, speed={self.speed}, name={self.name}")

    def arrive_callback(self, msg):
        self.has_arrive = 1
        rospy.loginfo("Arrived at destination!")
        if self.is_navigating and len(self.control.poses) > 0:
            self.current_target_index += 1
            if self.current_target_index >= len(self.control.poses):
                if self.control.loop == 1:  # 如果需要循环
                    self.current_target_index = 0
                else:
                    self.is_navigating = False
                    rospy.loginfo("Reached final destination")
                    return

            rospy.loginfo(f"Moving to next waypoint: {self.current_target_index}")
            self.send_next_waypoint()

    def send_next_waypoint(self):
        if not self.is_navigating or self.current_target_index >= len(self.control.poses):
            return

        # 创建PoseStamped消息发送给move_base
        target_pose = PoseStamped()
        target_pose.header.frame_id = "map"
        target_pose.header.stamp = rospy.Time.now()
        target_pose.pose = self.control.poses[self.current_target_index]

        rospy.loginfo(f"Sending waypoint: x={target_pose.pose.position.x}, y={target_pose.pose.position.y}")
        # 发布导航目标点
        self.move_base_pub.publish(target_pose)

    def handle_operation(self):
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rate.sleep()
            if self.receive == 0:
                self.has_arrive = 0
                continue

            rospy.loginfo(f"op={self.cKey}, status={self.status}")

            if self.cKey == 0:
                self.handle_map_loading()
            elif self.cKey == 1:
                self.handle_stop_navigation()
            elif self.cKey == 2:
                self.handle_waypoint_patrol()
            elif self.cKey == 3:
                self.handle_emergency_stop()

            self.receive = 0
            self.has_arrive = 0

    def handle_map_loading(self):
        if self.flag_initial == 0:
            self.ctrl_pub_initial.publish(String())
            self.flag_initial += 1

        move_cmd = [
            "cp", f"/home/robot/maps/{self.name}.pgm",
            "/home/robot/catkin_ws/src/wpb_home/wpb_home_tutorials/maps/map.pgm"
        ]
        subprocess.run(move_cmd)
        rospy.loginfo(f"Copied {self.name}.pgm")

        move_cmd_yaml = [
            "cp", f"/home/robot/maps/{self.name}.yaml",
            "/home/robot/catkin_ws/src/wpb_home/wpb_home_tutorials/maps/map.yaml"
        ]
        subprocess.run(move_cmd_yaml)
        rospy.loginfo(f"Copied {self.name}.yaml")

        self.ctrl_pub_start.publish(String())
        # self.image_process = subprocess.Popen(["rosrun", "Aft_g1", "image_detect"])
        self.status = 1

    def handle_stop_navigation(self):
        # subprocess.run(["rosnode", "kill", "image_detect"])
        self.status = 0
        self.is_navigating = False
        self.ctrl_pub_stop.publish(String())

        # 发送停止命令
        stop_twist = Twist()
        self.ctrl_pub_vel.publish(stop_twist)

        return_goal = NavControl()
        return_goal.op = 2
        return_goal.loop = 1
        zero_pose = Pose()
        zero_pose.orientation.w = 1
        return_goal.poses = [zero_pose]
        self.ctrl_pub_nav.publish(return_goal)

        while self.has_arrive != 1:
            rospy.sleep(0.1)

    def handle_waypoint_patrol(self):
        rospy.loginfo(f"Starting waypoint patrol with {len(self.control.poses)} waypoints")

        if len(self.control.poses) == 0:
            rospy.logwarn("No waypoints provided for patrol")
            return

        # 记录巡逻状态
        self.current_target_index = 0
        self.is_navigating = True

        # 打印waypoint信息便于调试
        for i, pose in enumerate(self.control.poses):
            rospy.loginfo(f"Waypoint {i}: x={pose.position.x}, y={pose.position.y}, z={pose.position.z}")
            rospy.loginfo(f"Orientation: x={pose.orientation.x}, y={pose.orientation.y}, z={pose.orientation.z}, w={pose.orientation.w}")

        # 确保所有姿态四元数都有有效值
        for i, pose in enumerate(self.control.poses):
            if pose.orientation.w == 0 and pose.orientation.x == 0 and pose.orientation.y == 0 and pose.orientation.z == 0:
                pose.orientation.w = 1.0  # 默认四元数

        # 将导航控制消息发布到ROS系统
        self.ctrl_pub_nav.publish(self.control)

        # 开始导航到第一个目标点
        self.send_next_waypoint()

    def handle_emergency_stop(self):
        self.is_navigating = False
        self.ctrl_pub_stop.publish(String())
        # 发送零速度命令立即停止
        stop_twist = Twist()
        self.ctrl_pub_vel.publish(stop_twist)
        rospy.loginfo("Emergency stop activated")

    def handle_named_location(self):
        name_vec = []
        pose_vec = []
        for i, name in enumerate(self.control.names):
            if name in self.command:
                name_vec.append(name)
                pose_vec.append(self.control.poses[i])

        if name_vec:
            self.control.op = 2
            self.control.names = name_vec
            self.control.poses = pose_vec
            self.ctrl_pub_nav.publish(self.control)

if __name__ == '__main__':
    try:
        node = NavControlNode()
        node.handle_operation()
    except rospy.ROSInterruptException:
        pass
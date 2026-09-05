#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import os
import subprocess
from geometry_msgs.msg import Twist
from Aft_g1.msg import MappingCmd
from map_files import save_map

class MapController:
    """
    地图控制器类，用于管理ROS机器人的建图操作
    """
    def __init__(self):
        # 速度相关参数
        self.linear_vel = 0.1
        self.angular_vel = 0.1
        self.k_vel = 3
        self.direction = 0
        self.flag = 0  # 建图开关标志
        self.speed = 0.0

        # 初始化ROS节点
        rospy.init_node('map_ctrl')

        self.cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.ctrl_sub = rospy.Subscriber('/map_ctrl', MappingCmd, self.callback)

        rospy.loginfo("Map Controller initialized!")

    def callback(self, cmd):
        """
        处理接收到的地图控制命令

        参数:
            cmd: MappingCmd消息，包含控制命令或方向信息
        """
        rospy.loginfo("Callback function called!")

        if not cmd.isCmd:
            # 处理方向控制命令
            rospy.loginfo("Data is direction!")
            if self.flag and (cmd.direction >= 0 and cmd.direction <= 6):
                # 创建Twist消息
                base_cmd = Twist()
                base_cmd.linear.x = 0
                base_cmd.linear.y = 0
                base_cmd.angular.z = 0

                # 根据方向设置速度
                if cmd.direction == 1:
                    base_cmd.linear.x = cmd.speed
                elif cmd.direction == 2:
                    base_cmd.linear.x = -cmd.speed
                elif cmd.direction == 3:
                    base_cmd.linear.y = cmd.speed
                elif cmd.direction == 4:
                    base_cmd.linear.y = -cmd.speed
                elif cmd.direction == 5:
                    base_cmd.angular.z = cmd.speed
                elif cmd.direction == 6:
                    base_cmd.angular.z = -cmd.speed

                # 发布速度命令
                self.cmd_vel_pub.publish(base_cmd)
        else:
            # 处理建图相关命令
            rospy.loginfo("Data is command!")
            rospy.loginfo(f"cmd.cmdId = {cmd.cmdId}, flag = {self.flag}")

            # 命令1: 开始建图
            if cmd.cmdId == 1:
                self.flag = 1
                rospy.loginfo("=============================================")
                subprocess.Popen(["roslaunch", "Aft_g1", "gmapping.launch"])
                rospy.loginfo("Mapping started!")

            # 命令2: 取消建图
            elif self.flag and cmd.cmdId == 2:
                rospy.loginfo("Mapping cancelled!")
                self.flag = 0
                # subprocess.call(["rosnode", "kill", "gmapping"])
            # 命令3: 保存地图
            elif self.flag and cmd.cmdId == 3:
                save_map(cmd.graphName, rospy.get_param("~maps_dir", "/home/robot/maps"))
                rospy.loginfo("Map saved!")

    def run(self):
        """
        运行地图控制器，开始接收和处理命令
        """
        rospy.loginfo("Map Controller is running...")
        rospy.spin()


if __name__ == '__main__':
    try:
        controller = MapController()
        controller.run()
    except rospy.ROSInterruptException:
        print("[ERROR] map ctrl error!")

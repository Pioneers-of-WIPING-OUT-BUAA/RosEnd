#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry
from actionlib_msgs.msg import GoalStatusArray, GoalStatus
from std_msgs.msg import String

from Aft_g1.msg import ErrorMsg

class NavErrorMonitor:
    """
    导航错误监控节点，用于检测机器人在导航过程中是否长时间停止移动
    """
    def __init__(self):
        rospy.init_node('nav_error')

        self.last_position = None
        self.new_position = None
        self.flag = 0
        self.time_stop = 0
        self.isActive = 0

        self.pub = rospy.Publisher('/nav/stop', String, queue_size=1000)
        self.pub_error = rospy.Publisher('/error', ErrorMsg, queue_size=1000)

        # 创建订阅者
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/move_base/status', GoalStatusArray, self.status_callback)

        self.rate = rospy.Rate(10)

        rospy.loginfo("导航错误监控节点已启动")

    def status_callback(self, msg):
        for status in msg.status_list:
            if status.status == GoalStatus.ACTIVE:
                self.isActive = 1
            else:
                self.isActive = 0

    # 监控机器人位置变化
    def odom_callback(self, msg):
        if self.flag == 0:
            self.flag = 1
            self.last_position = msg
            return
        else:
            self.last_position = self.new_position
            self.new_position = msg

        # 获取当前位置
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        # 计算位置变化
        if self.last_position is not None:
            old_x = self.last_position.pose.pose.position.x
            old_y = self.last_position.pose.pose.position.y
            old_z = self.last_position.pose.pose.position.z
        else: # wxm: I guess so
            old_x = 0
            old_y = 0
            old_z = 0
        new_x = self.new_position.pose.pose.position.x
        new_y = self.new_position.pose.pose.position.y
        new_z = self.new_position.pose.pose.position.z

        # 计算位移平方和
        sum_squared_diff = (old_x - new_x) ** 2 + (old_y - new_y) ** 2

        # 检测是否停止移动
        if sum_squared_diff < 0.0004:
            if self.isActive == 0:
                self.time_stop = 0
            else:
                self.time_stop += 1
                if self.time_stop >= 70:
                    rospy.logwarn("机器人超过7秒未移动！")
                    stop_msg = String()
                    stop_msg.data = "a"
                    self.pub.publish(stop_msg)

                    error_msg = ErrorMsg()
                    error_msg.type = 0
                    error_msg.message = "导航过程中无法找到正确路径，导航停止"
                    self.pub_error.publish(error_msg)

                    rospy.loginfo("已发送导航停止信号")
        else:
            self.time_stop = 0
            rospy.logdebug("位移平方和: %f", sum_squared_diff)

    def run(self):
        rospy.loginfo("导航错误监控节点开始运行")
        while not rospy.is_shutdown():
            self.rate.sleep()

if __name__ == '__main__':
    try:
        monitor = NavErrorMonitor()
        monitor.run()
    except rospy.ROSInterruptException:
        print("[ERROR] nav error error!")

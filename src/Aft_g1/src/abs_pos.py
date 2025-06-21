#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from nav_msgs.msg import Odometry, MapMetaData
from geometry_msgs.msg import PointStamped, Point
from std_msgs.msg import Char


class RobotPositionNode:
    """
    机器人位置节点类
    负责计算并发布机器人在地图中的绝对位置
    """

    def __init__(self):
        rospy.init_node('robot_position_node')

        self.current_pose = PointStamped()
        self.map_metadata = MapMetaData()
        self.robot_position = Point()

        self.position_publisher = rospy.Publisher('robot_absolute_add', Point, queue_size=1)
        self.send_place_pub = rospy.Publisher('/nav/absolute_place', Point, queue_size=1)

        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/map_metadata', MapMetaData, self.metadata_callback)
        rospy.Subscriber('/nav/need_send_place', Char, self.place_callback)

        self.rate = rospy.Rate(10)

    def odom_callback(self, odom):
        """
        机器人里程计数据回调函数

        Args:
            odom: 里程计消息
        """
        self.current_pose.header = odom.header
        self.current_pose.point = odom.pose.pose.position

    def metadata_callback(self, metadata):
        """
        地图元数据回调函数

        Args:
            metadata: 地图元数据消息
        """
        self.map_metadata = metadata

    def place_callback(self, msg):
        """
        位置请求回调函数

        Args:
            msg: 字符消息
        """
        self.send_place_pub.publish(self.robot_position)

    def calculate_position(self):
        """计算机器人在地图中的绝对位置"""
        self.robot_position.x = self.current_pose.point.x - self.map_metadata.origin.position.x
        self.robot_position.y = self.current_pose.point.y - self.map_metadata.origin.position.y

    def publish_position(self):
        """发布机器人位置信息"""
        self.position_publisher.publish(self.robot_position)
        rospy.loginfo("x=%f, y=%f", self.current_pose.point.x, self.current_pose.point.y)

    def run(self):
        while not rospy.is_shutdown():
            self.calculate_position()
            self.publish_position()
            self.rate.sleep()


if __name__ == '__main__':
    try:
        node = RobotPositionNode()
        node.run()
    except rospy.ROSInterruptException:
        pass
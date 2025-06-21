#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import time
import math
import tf
from sensor_msgs.msg import Imu
from Aft_g1.msg import ErrorMsg


class PoseErrorDetector:
    """
    姿态错误检测器类，用于监控IMU数据，检测机器人姿态异常
    """
    def __init__(self):
        # 初始化阈值常量
        self.MIN_IMU = 0.1

        rospy.init_node('pose_error')

        self.pub_error = rospy.Publisher('/error', ErrorMsg, queue_size=1000)
        rospy.Subscriber('imu/data', Imu, self.imu_callback, queue_size=100)

    def imu_callback(self, msg):
        """
        IMU数据回调处理函数

        参数:
            msg: 接收到的IMU消息
        """
        # 检测消息包中四元数数据是否存在
        if msg.orientation_covariance[0] < 0:
            return

        # 四元数转成欧拉角
        quaternion = (
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )

        # 使用tf获取欧拉角
        roll, pitch, yaw = tf.transformations.euler_from_quaternion(quaternion)

        # 弧度换算成角度
        roll_deg = roll * 180 / math.pi
        pitch_deg = pitch * 180 / math.pi
        yaw_deg = yaw * 180 / math.pi

        rospy.loginfo("滚转= %.0f 俯仰= %.0f 朝向= %.0f", roll_deg, pitch_deg, yaw_deg)
        print(pitch_deg)

        # 检测姿态信息是否正常
        if pitch_deg > self.MIN_IMU or pitch_deg < -self.MIN_IMU:
            rospy.loginfo("[ERROR] 姿态异常")
            # 创建error信息
            self.publish_error_message("机器人姿态异常")

    def publish_error_message(self, message):
        """
        发布错误消息

        参数:
            message: 错误信息内容
        """
        error_message = ErrorMsg()
        error_message.type = 1
        error_message.message = message
        self.pub_error.publish(error_message)
        time.sleep(1)


if __name__ == "__main__":
    detector = PoseErrorDetector()
    try:
        rospy.spin()
    except KeyboardInterrupt:
        rospy.loginfo("姿态错误检测节点已关闭")
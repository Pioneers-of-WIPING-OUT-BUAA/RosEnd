#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
import tf
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped


class TransformPosition:
    """
    TransformPosition类用于监听TF变换并发布机器人位姿

    该类监听/map和/base_link之间的TF变换，
    获取机器人在map坐标系中的位置和方向，
    并在接收到特定触发消息时将位姿发布到/f-16*.log
[INFO] [Kinect2Bridge::main] color processing: ~15.6376ms (~63.9484Hz) publishing rate: ~23.6208Hz
[Info] [TurboJpegRgbPacketProcessor] avg. time: 35.5881ms -> ~28.0993Hz
[INFO] [Kinect2Bridge::main] color processing: ~14.5004ms (~68.9638Hz) publishing rate: ~24.9447Hz
[WARN] [1750256604.142401]: Sound command issued, but no node is subscribed toinitialpose话题
    """

    def __init__(self):
        rospy.init_node('transform_position')

        # 创建TF监听器
        self.listener = tf.TransformListener()

        # 创建位姿消息
        self.pose_msg = PoseWithCovarianceStamped()
        self.pose_msg.header.frame_id = "map"

        # 创建发布者和订阅者
        self.pub = rospy.Publisher('/initialpose', PoseWithCovarianceStamped, queue_size=1000)
        self.sub = rospy.Subscriber('/pub_initial', String, self.sub_callback)

        # 等待TF数据发布
        rospy.sleep(0.5)

    def sub_callback(self, msg):
        """
        订阅/pub_initial话题的回调函数

        当接收到消息时，发布当前位姿到/initialpose话题

        Args:
            msg: 接收到的消息
        """
        self.pub.publish(self.pose_msg)

    def update_pose(self):
        """更新机器人位姿信息"""
        try:
            # 等待并查询变换
            self.listener.waitForTransform("/map", "/base_link", rospy.Time(0), rospy.Duration(3.0))
            (trans, rot) = self.listener.lookupTransform("/map", "/base_link", rospy.Time(0))

            # 更新位姿消息
            self.pose_msg.header.stamp = rospy.Time.now()

            # 设置位置
            self.pose_msg.pose.pose.position.x = trans[0]
            self.pose_msg.pose.pose.position.y = trans[1]
            self.pose_msg.pose.pose.position.z = trans[2]

            # 设置方向
            self.pose_msg.pose.pose.orientation.x = rot[0]
            self.pose_msg.pose.pose.orientation.y = rot[1]
            self.pose_msg.pose.pose.orientation.z = rot[2]
            self.pose_msg.pose.pose.orientation.w = rot[3]

            # 输出位置信息
            rospy.loginfo(
                "Robot is at (x, y, z) = (%.2f, %.2f, %.2f), (x, y, z, w) = (%.2f, %.2f, %.2f, %.2f)",
                trans[0], trans[1], trans[2], rot[0], rot[1], rot[2], rot[3]
            )

            return True
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException) as e:
            rospy.logerr("%s", str(e))
            return False

    def run(self):
        rate = rospy.Rate(1)

        while not rospy.is_shutdown():
            self.update_pose()
            rate.sleep()


if __name__ == '__main__':
    try:
        transform_position = TransformPosition()
        transform_position.run()
    except rospy.ROSInterruptException:
        print("[ERROR] pose transform error!")

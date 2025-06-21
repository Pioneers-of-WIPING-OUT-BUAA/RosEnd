#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from geometry_msgs.msg import Pose
from geometry_msgs.msg import PoseWithCovarianceStamped
from Aft_g1.srv import PoseSrv, PoseSrvResponse


class PoseServer:
    def __init__(self):
        self.pose = Pose()

        rospy.init_node("pose_server")

        # 创建服务和订阅
        self.service = rospy.Service("/cur_pose", PoseSrv, self.current_pose_callback)
        self.subscriber = rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self.update_pose_callback)

        rospy.loginfo("位姿服务器已启动")

    def show_pose(self, pose):
        rospy.loginfo("position:")
        rospy.loginfo("  x: {:.2f}".format(pose.position.x))
        rospy.loginfo("  y: {:.2f}".format(pose.position.y))
        rospy.loginfo("  z: {:.2f}".format(pose.position.z))
        rospy.loginfo("orientation:")
        rospy.loginfo("  x: {:.2f}".format(pose.orientation.x))
        rospy.loginfo("  y: {:.2f}".format(pose.orientation.y))
        rospy.loginfo("  z: {:.2f}".format(pose.orientation.z))
        rospy.loginfo("  w: {:.2f}".format(pose.orientation.w))

    def current_pose_callback(self, req):
        """处理位姿服务请求的回调函数

        Args:
            req: 服务请求

        Returns:
            PoseSrvResponse: 包含当前位姿的响应
        """
        response = PoseSrvResponse()
        response.pose = self.pose

        rospy.loginfo("收到位姿服务请求")
        self.show_pose(self.pose)

        return response

    def update_pose_callback(self, data):
        """更新当前位姿的回调函数

        Args:
            data: 包含新位姿信息的消息
        """
        self.pose = data.pose.pose

        rospy.loginfo("位姿已更新!")
        self.show_pose(self.pose)

    def run(self):
        """运行位姿服务器"""
        rospy.spin()


if __name__ == "__main__":
    try:
        server = PoseServer()
        server.run()
    except rospy.ROSInterruptException:
        print("[ERROR] cur_pose serve error!")

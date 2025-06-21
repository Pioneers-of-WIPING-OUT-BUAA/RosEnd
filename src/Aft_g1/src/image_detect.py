#!/usr/bin/env python3

import rospy
import cv2
import numpy as np
import time

from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from Aft_g1.msg import SoundMsg

import rospy
import cv2
import numpy as np
from abc import abstractmethod


class ColorDetector:
    def __init__(self):
        # 定义颜色范围（HSV空间）
        self.color_ranges = {
            'red1': (np.array([0, 100, 100]), np.array([10, 255, 255])),
            'red2': (np.array([170, 100, 100]), np.array([180, 255, 255])),
            'green': (np.array([35, 100, 100]), np.array([85, 255, 255])),
            'blue': (np.array([100, 100, 100]), np.array([130, 255, 255]))
        }
        # 设置检测阈值
        self.threshold = 1000

    def detect(self, image):
        """
        检测图像中的主要颜色
        返回值说明：
        0: 没有检测到物体
        1: 检测到红色物体
        2: 检测到绿色物体
        3: 检测到蓝色物体
        4: 检测到其他物体
        """
        try:
            # 转换到HSV颜色空间
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

            # 检测各种颜色
            red_mask1 = cv2.inRange(hsv, self.color_ranges['red1'][0], self.color_ranges['red1'][1])
            red_mask2 = cv2.inRange(hsv, self.color_ranges['red2'][0], self.color_ranges['red2'][1])
            red_mask = cv2.bitwise_or(red_mask1, red_mask2)
            green_mask = cv2.inRange(hsv, self.color_ranges['green'][0], self.color_ranges['green'][1])
            blue_mask = cv2.inRange(hsv, self.color_ranges['blue'][0], self.color_ranges['blue'][1])

            # 计算每种颜色的像素数量
            red_pixels = cv2.countNonZero(red_mask)
            green_pixels = cv2.countNonZero(green_mask)
            blue_pixels = cv2.countNonZero(blue_mask)

            # 判断检测结果
            if red_pixels > self.threshold and red_pixels > green_pixels and red_pixels > blue_pixels:
                return 1
            elif green_pixels > self.threshold and green_pixels > red_pixels and green_pixels > blue_pixels:
                return 2
            elif blue_pixels > self.threshold and blue_pixels > red_pixels and blue_pixels > green_pixels:
                return 3
            elif max(red_pixels, green_pixels, blue_pixels) > self.threshold:
                return 4
            else:
                return 0
        except Exception as e:
            rospy.logerr(f"Error in color detection: {e}")
            return 0
        
class ImageDetector:
    def __init__(self):
        # 初始化ROS节点
        rospy.init_node('image_detect', anonymous=True)

        # 创建CV桥接器
        self.bridge = CvBridge()

        # 创建颜色检测器实例
        self.color_detector = ColorDetector()

        # 创建发布者和订阅者
        self.pub = rospy.Publisher('/sound_detect', SoundMsg, queue_size=10)
        self.sub = rospy.Subscriber('/kinect2/hd/image_color_rect', Image, self.image_callback)

        # 上次处理时间
        self.last_time = 0

        rospy.loginfo("Image detector node initialized")

    def image_callback(self, msg):
        # 控制处理频率
        current_time = time.time()
        if current_time - self.last_time < 2:
            if current_time - self.last_time < 1:
                time.sleep(1)
            return
        self.last_time = current_time

        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

            # 计算图像大小和缩放比例
            height, width = cv_image.shape[:2]
            hw = width * height
            scale = (400000.0 / hw) ** 0.5

            # 调整图像大小
            if scale < 1:
                image_compressed = cv2.resize(cv_image, None, fx=scale, fy=scale,
                                           interpolation=cv2.INTER_LINEAR)
            else:
                image_compressed = cv_image

            # 直接进行颜色检测
            result = self.color_detector.detect(image_compressed)

            # 发布结果消息
            msg = SoundMsg()
            msg.op = result
            self.pub.publish(msg)

            rospy.loginfo(f"Detect result: {result}")

        except CvBridgeError as e:
            rospy.logerr(f"CV Bridge error: {e}")
        except Exception as e:
            rospy.logerr(f"Error in image_callback: {e}")

if __name__ == '__main__':
    try:
        detector = ImageDetector()
        rospy.spin()
    except rospy.ROSInterruptException:
        print("[ERROR] image_detect start error!")
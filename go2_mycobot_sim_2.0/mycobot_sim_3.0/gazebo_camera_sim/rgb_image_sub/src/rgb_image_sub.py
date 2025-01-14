#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
import cv2
import numpy as np
from collections import deque

# 设定红色阈值，HSV空间
redLower = np.array([0, 80, 50])
redUpper = np.array([8, 255, 220])
mybuffer = 64  # 初始化追踪点的列表
pts = deque(maxlen=mybuffer)

def image_callback(msg):
    bridge = CvBridge()
    try:
        # 将ROS图像消息转换为OpenCV图像
        cv_image = bridge.imgmsg_to_cv2(msg, "bgr8")
    except CvBridgeError as e:
        print(e)
        return

    # 转到HSV空间
    hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)
    # 根据阈值构建掩膜
    mask = cv2.inRange(hsv, redLower, redUpper)
    # 腐蚀操作
    mask = cv2.erode(mask, None, iterations=2)
    # 膨胀操作，去除噪点
    mask = cv2.dilate(mask, None, iterations=2)
    # 轮廓检测
    cnts = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)[-2]
    center = None  # 初始化瓶盖圆形轮廓质心

    # 如果存在轮廓
    if len(cnts) > 0:
        # 找到面积最大的轮廓
        c = max(cnts, key=cv2.contourArea)
        # 确定面积最大的轮廓的外接圆
        ((center_x, center_y), radius) = cv2.minEnclosingCircle(c)
        # 计算轮廓的矩
        M = cv2.moments(c)
        # 计算质心
        center = (int(M['m10'] / M['m00']), int(M['m01'] / M['m00']))
        cv2.circle(cv_image, (int(center_x), int(center_y)), int(radius), (0, 255, 255), 2)
        cv2.circle(cv_image, center, 5, (0, 0, 255), -1)
        print('红色色块的中心坐标', center)

    # 显示图像
    cv2.imshow("RGB Image", cv_image)
    cv2.waitKey(30)

def main():
    rospy.init_node('rgb_image_subscriber', anonymous=True)
    
    # 订阅RGB图像话题
    rospy.Subscriber("/camera/color/image_raw", Image, image_callback)
    
    # 保持节点运行
    rospy.spin()

if __name__ == '__main__':
    main()

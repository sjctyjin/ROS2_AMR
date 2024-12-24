#!/usr/bin/env python3
#coding=utf-8

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

import cv2
import numpy as np
import time

'''
	cv2.VideoCapture 视频读取，发布图片话题
        从摄像头或视频文件
'''


'''
    发布图片话题：
    encoding为图片的编码方式，要与图片对应，opencv读取的彩色图片为"bgr8"，灰度图片为"mono8"，彩色透明图为"bgra8"
'''
def publishImage(image, publisher, bridge, encoding="bgr8"):
    
    image_temp = bridge.cv2_to_imgmsg(image, encoding)
    publisher.publish(image_temp) 


def main(args=None):                  # ROS2节点主入口main函数
    rclpy.init(args=args)             # ROS2 Python接口初始化
    node = Node("Videoread")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("Videoread node start !!!")

    bridge = CvBridge()
    videoPub = node.create_publisher(Image, '/camera/image_raw', 1) #创建图片话题发布器

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"

    # 读取视频文件
    vc = cv2.VideoCapture(baseDir + 'data/vtest.avi') 

    # # 读取摄像头
    # vc = cv2.VideoCapture(0) #0为设备号，对应的是摄像头video0，若你的设备为video1，则使用1

    if vc.isOpened(): # 检查是否打开正确
        open, frame = vc.read()
        print("open success ...")
    else:
        open = False
        print("打开失败，摄像头设备或视频文件不存在")

    count = 0
    while open and rclpy.ok():
        ret, frame = vc.read() # 读取一帧图片
        if frame is None:
            break
        if ret == True:
            
            count += 1
            print("pubulish",count)
            # frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) # 将视频图转成灰度
            if len(frame.shape) == 3:
                encoding = 'bgr8'
            else:
                encoding = "mono8"
            publishImage(frame,videoPub, bridge, encoding)
            cv2.waitKey(200)#等待100ms 
    vc.release()
    
    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口
    
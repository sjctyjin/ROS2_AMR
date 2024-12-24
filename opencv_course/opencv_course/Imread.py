#!/usr/bin/env python3
# -*- coding: utf-8 -*-

'''
    opencv图片基本操作：读、写、显示等
'''
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Header
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge

import cv2
import numpy as np
import time


'''
    发布图片话题：
    encoding为图片的编码方式，要与图片对应，opencv读取的彩色图片为"bgr8"，灰度图片为"mono8"，彩色透明图为"bgra8"
'''
def publishImage(image, publisher, bridge, encoding="bgr8"):
    
    image_temp = bridge.cv2_to_imgmsg(image, encoding)
    publisher.publish(image_temp) 


def main(args=None):                  # ROS2节点主入口main函数
    rclpy.init(args=args)             # ROS2 Python接口初始化
    node = Node("ImgRead")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("ImgRead node start !!!")

    bridge = CvBridge()
    imgRawPub = node.create_publisher(Image, '/image/raw', 1) #创建图片话题发布器

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"
       
    img = cv2.imread(baseDir + 'data/lena.jpg')
    if img is None:
        print("读取图片失败，请检查图片路径是否正确！")
    #图像的显示,也可以创建多个窗口
    # cv2.imshow('image',img)
    # cv2.waitKey(0) #等待时间，毫秒级，会阻塞程序，0表示一直阻塞，直到关闭窗口

    # 保存图片
    cv2.imwrite(baseDir + 'rst/lena.png',img)
    print("img.shape: ",img.shape) # 打印图片尺寸
    print("img.dtype: ",img.dtype) # 打印图片数据类型
    #将图像调整大小
    img1 = cv2.resize(img, (500, 414))
    print("img1.shape: ",img1.shape) # 打印图片尺寸
    # imgResizePub=rospy.Publisher('/image/resize',Image,queue_size=1) #创建图片话题发布器
    imgResizePub = node.create_publisher(Image, '/image/resize', 1)

    #将图像通道分离与组合
    b,g,r=cv2.split(img)
    img2=cv2.merge((r,g,b)) # 将分离的图像通道打乱后，重新组合，图片会与原图不同
    # imgMergePub=rospy.Publisher('/image/merge',Image,queue_size=1) #创建图片话题发布器
    imgMergePub = node.create_publisher(Image, '/image/merge', 1)

    # 将上面图像操作的结果以图片的话题形式发布，一秒一次
    count = 0
    while rclpy.ok():
        publishImage(img,imgRawPub,bridge)
        publishImage(img1,imgResizePub,bridge)
        publishImage(img2,imgMergePub,bridge)
        time.sleep(1)
        count += 1
        print ("publish ",count)

    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口




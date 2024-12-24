#!/usr/bin/env python3
#coding=utf-8

'''
图像二值化函数cv2.threshold()，和OTSU自动阈值法
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
    node = Node("Im_threshold")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("Im_threshold node start !!!")

    bridge = CvBridge()

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"
    
    img = cv2.imread(baseDir + "data/lena.jpg")
    if img is None:
        print("读取图片失败，请检查图片路径是否正确！")
        exit()
    else:
        print("读取图片成功！")
        print("img.shape:",img.shape)
    img_gray = cv2.cvtColor(img,cv2.COLOR_BGR2GRAY)
    # cv2.imshow('img_gray',img_gray)
    imgPub = node.create_publisher(Image, '/image/img_raw', 1) #创建图片话题发布器


    ret, thresh1 = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY)
    ret, thresh2 = cv2.threshold(img_gray, 127, 255, cv2.THRESH_BINARY_INV)
    ret, thresh3 = cv2.threshold(img_gray, 127, 255, cv2.THRESH_TRUNC)
    ret, thresh4 = cv2.threshold(img_gray, 127, 255, cv2.THRESH_TOZERO)
    ret, thresh5 = cv2.threshold(img_gray, 127, 255, cv2.THRESH_TOZERO_INV)

    # cv2.imshow('THRESH_BINARY', thresh1)
    # cv2.imshow('THRESH_BINARY_INV', thresh2)
    # cv2.imshow('THRESH_TRUNC', thresh3)
    # cv2.imshow('THRESH_TOZERO', thresh4)
    # cv2.imshow('THRESH_TOZERO_INV', thresh5)
    # cv2.waitKey(0)

    #OTSU阈值分割
    ret, thresh6 = cv2.threshold(img_gray, 0, 255, cv2.THRESH_OTSU)  #方法选择为THRESH_OTSU
    # cv2.imshow('THRESH_OTSU', thresh6)
    # cv2.waitKey(0)

    thresh1Pub = node.create_publisher(Image, '/image/THRESH_BINARY', 1) #创建图片话题发布器
    thresh2Pub = node.create_publisher(Image, '/image/THRESH_BINARY_INV', 1) #创建图片话题发布器
    thresh3Pub = node.create_publisher(Image, '/image/THRESH_TRUNC', 1) #创建图片话题发布器
    thresh4Pub = node.create_publisher(Image, '/image/THRESH_TOZERO', 1) #创建图片话题发布器
    thresh5Pub = node.create_publisher(Image, '/image/THRESH_TOZERO_INV', 1) #创建图片话题发布器
    thresh6Pub = node.create_publisher(Image, '/image/THRESH_OTSU', 1) #创建图片话题发布器

    # 将上面图像操作的结果以图片的话题形式发布，一秒一次
    count = 0
    while rclpy.ok():
        publishImage(img_gray, imgPub, bridge,"mono8")
        publishImage(thresh1, thresh1Pub, bridge,"mono8")
        publishImage(thresh2, thresh2Pub, bridge,"mono8")
        publishImage(thresh3, thresh3Pub, bridge,"mono8")
        publishImage(thresh4, thresh4Pub, bridge,"mono8")
        publishImage(thresh5, thresh5Pub, bridge,"mono8")
        publishImage(thresh6, thresh6Pub, bridge,"mono8")

        time.sleep(1)
        count += 1
        print("publish ",count)

    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口

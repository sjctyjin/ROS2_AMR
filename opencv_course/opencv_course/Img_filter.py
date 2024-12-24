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
    图片滤波处理算法：
        均值滤波，中值滤波，高斯滤波和双边滤波
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
    node = Node("Img_filter")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("Img_filter node start !!!")

    bridge = CvBridge()

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"

    #读取图像
    img = cv2.imread(baseDir + 'data/lenaNoise.png')
    if img is None:
        print("读取图片失败，请检查图片路径是否正确！")
        exit()
    else:
        print("读取图片成功！")
        print("img.shape:",img.shape)

    imgPub= node.create_publisher(Image,'/image/img_raw', 1) #创建图片话题发布器
    # cv2.imshow('img', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    #均值滤波
    blurImg = cv2.blur(img, (3, 3))
    blurImgPub = node.create_publisher(Image, '/image/blur', 1) 
    # cv2.imshow('blur', blurImg)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    #中值滤波
    medianImg = cv2.medianBlur(img, 5)  # 中值滤波
    medianImgPub = node.create_publisher(Image, '/image/median', 1) 
    # cv2.imshow('median', medianImg)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    #高斯滤波
    gaussianImg = cv2.GaussianBlur(img, (5, 5), 1)
    gaussianImgPub = node.create_publisher(Image, '/image/gaussian', 1) 
    # cv2.imshow('gaussianImg', gaussianImg)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    #双边滤波
    #后面两个数字：空间高斯函数标准差，灰度值相似性标准差
    bilateralFilterImg = cv2.bilateralFilter(img,9,210,210)
    bilateralFilterImgPub = node.create_publisher(Image, '/image/bilateral_filter', 1) 

    # cv2.imshow('bilateralFilterImg', bilateralFilterImg)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()

    # 将上面图像操作的结果以图片的话题形式发布，一秒一次
    count = 0
    while rclpy.ok():
        publishImage(img,imgPub ,bridge)
        publishImage(blurImg,blurImgPub ,bridge)
        publishImage(medianImg,medianImgPub ,bridge)
        publishImage(gaussianImg,gaussianImgPub ,bridge)
        publishImage(bilateralFilterImg,bilateralFilterImgPub ,bridge)
        time.sleep(1)
        count += 1
        print("publish ",count)

    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口


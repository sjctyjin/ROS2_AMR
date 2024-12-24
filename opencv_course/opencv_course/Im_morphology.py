#!/usr/bin/env python3
#coding=utf-8

'''
图像形态学操作运算:
    膨胀运算、腐蚀运算、开操作、闭操作、梯度运算、顶帽和黑帽
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
    node = Node("Im_morphology")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("Im_morphology node start !!!")

    bridge = CvBridge()

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"
    
    img = cv2.imread(baseDir + 'data/blocks.jpg')
    # cv2.imshow('img', img)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    if img is None:
        print("读取图片失败，请检查图片路径是否正确！")
        exit()
    else:
        print("读取图片成功！")
        print("img.shape:",img.shape)

    imgPub = node.create_publisher(Image, '/image/img_raw', 1) #创建图片话题发布器

    #腐蚀操作
    kernel = np.ones((3,3),np.uint8)
    dige_erosion = cv2.erode(img,kernel,iterations = 1)
    # cv2.imshow('erosion', dige_erosion)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    erodeImgPub = node.create_publisher(Image, '/image/erode', 1) #创建图片话题发布器

    #膨胀操作
    kernel = np.ones((3,3),np.uint8)
    dige_dilate = cv2.dilate(img,kernel,iterations = 1)
    # cv2.imshow('dilate', dige_dilate)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    dilateImgPub = node.create_publisher(Image, '/image/dilate', 1) #创建图片话题发布器

    #开操作
    # 开：先腐蚀，再膨胀
    kernel = np.ones((5,5),np.uint8)
    opening = cv2.morphologyEx(img, cv2.MORPH_OPEN, kernel)
    # cv2.imshow('opening', opening)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    openingImgPub = node.create_publisher(Image, '/image/opening', 1) #创建图片话题发布器 

    #闭操作
    # 闭：先膨胀，再腐蚀
    kernel = np.ones((7,7),np.uint8)
    closing = cv2.morphologyEx(img, cv2.MORPH_CLOSE, kernel)
    # cv2.imshow('closing', closing)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    closingImgPub = node.create_publisher(Image, '/image/closing', 1) #创建图片话题发布器

    #图像的形态学梯度
    gradient = cv2.morphologyEx(img, cv2.MORPH_GRADIENT, kernel)
    # cv2.imshow('gradient', gradient)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    gradientImgPub = node.create_publisher(Image, '/image/gradient', 1) #创建图片话题发布器

    #图像的顶帽
    kernel = np.ones((7,7),np.uint8)
    tophat = cv2.morphologyEx(img, cv2.MORPH_TOPHAT, kernel)
    # cv2.imshow('tophat', tophat)
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    tophatImgPub = node.create_publisher(Image, '/image/tophat', 1) #创建图片话题发布器

    #图像的黑帽
    kernel = np.ones((7,7),np.uint8)
    blackhat  = cv2.morphologyEx(img,cv2.MORPH_BLACKHAT, kernel)
    # cv2.imshow('blackhat ', blackhat )
    # cv2.waitKey(0)
    # cv2.destroyAllWindows()
    blackhatImgPub = node.create_publisher(Image, '/image/blackhat', 1) #创建图片话题发布器

    # 将上面图像操作的结果以图片的话题形式发布，一秒一次
    count = 0
    while rclpy.ok():
        publishImage(img, imgPub, bridge)
        publishImage(dige_erosion, erodeImgPub, bridge)
        publishImage(dige_dilate, dilateImgPub, bridge)
        publishImage(opening, openingImgPub, bridge)
        publishImage(closing, closingImgPub, bridge)
        publishImage(gradient,gradientImgPub, bridge)
        publishImage(tophat,tophatImgPub, bridge)
        publishImage(blackhat,blackhatImgPub, bridge)

        time.sleep(1)
        count += 1
        print("publish ",count)

    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口   


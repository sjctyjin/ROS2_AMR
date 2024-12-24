#!/usr/bin/env python
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
    图像的边缘处理算子
        一阶微分算子和二阶微分算子，以及Canny边缘检测算法
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
    node = Node("image_edge")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("image_edge node start !!!")

    bridge = CvBridge()

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"
    
    #提取图像
    img = cv2.imread(baseDir + 'data/lena.jpg',cv2.IMREAD_GRAYSCALE)
    # cv2.imshow('img',img)
    # cv2.waitKey(0)
    if img is None:
        print("读取图片失败，请检查图片路径是否正确！")
        exit()
    else:
        print("读取图片成功！")
        print("img.shape:",img.shape)

    imgPub = node.create_publisher(Image, '/image/img_raw', 1) 

    #提取图像sobel算子
    sobelx = cv2.Sobel(img,cv2.CV_64F,1,0,ksize=3)
    sobely = cv2.Sobel(img,cv2.CV_64F,0,1,ksize=3)
    sobelx = cv2.convertScaleAbs(sobelx)
    sobely = cv2.convertScaleAbs(sobely)
    sobelxy =  cv2.addWeighted(sobelx,0.5,sobely,0.5,0)
    # cv2.imshow('sobelxy',sobelxy)
    # cv2.waitKey(0)
    sobelxyPub = node.create_publisher(Image, '/image/sobelxy', 1) 

    #提取图像scharr算子
    scharrx = cv2.Scharr(img,cv2.CV_64F,1,0)
    scharry = cv2.Scharr(img,cv2.CV_64F,0,1)
    scharrx = cv2.convertScaleAbs(scharrx)
    scharry = cv2.convertScaleAbs(scharry)
    scharrxy =  cv2.addWeighted(scharrx,0.5,scharry,0.5,0)
    # cv2.imshow('scharrxy',scharrxy)
    # cv2.waitKey(0)
    scharrxyPub = node.create_publisher(Image, '/image/scharrxy', 1) 



    #提取图像laplacian算子
    laplacian = cv2.Laplacian(img,cv2.CV_64F)
    laplacian = cv2.convertScaleAbs(laplacian)
    # cv2.imshow('laplacian',laplacian)
    # cv2.waitKey(0)
    laplacianPub = node.create_publisher(Image, '/image/laplacian', 1) 


    #整合三种边缘提取算子
    res = np.hstack((sobelxy,scharrxy,laplacian)) #将这三张图按行拼成一张图，便于比较
    # cv2.imshow('res',res)
    # cv2.waitKey(0)
    resPub = node.create_publisher(Image, '/image/res', 1) 


    v1=cv2.Canny(img,80,150)
    v2=cv2.Canny(img,50,100)

    res1 = np.hstack((v1,v2)) # 将这两个不同参数的边缘检测结果按行拼成一张图，便于比较
    # cv2.imshow('res1',res1)
    # cv2.waitKey(0)
    res1Pub = node.create_publisher(Image, '/image/canny', 1) 


    # 将上面图像操作的结果以图片的话题形式发布，一秒一次
    count = 0
    while rclpy.ok():
        publishImage(img, imgPub, bridge, "mono8")
        publishImage(sobelxy, sobelxyPub, bridge,"mono8")
        publishImage(scharrxy, scharrxyPub, bridge, "mono8")
        publishImage(laplacian, laplacianPub, bridge, "mono8")
        publishImage(res,resPub, bridge, "mono8")
        publishImage(res1,res1Pub, bridge, "mono8")
        time.sleep(1)
        count += 1
        print("publish ",count)

    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口
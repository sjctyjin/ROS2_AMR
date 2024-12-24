#!/usr/bin/env python3
#coding=utf-8

'''
图像卷积操作
    自编程实现图像卷积操作和 opencv自带的卷积操作函数cv2.filter2D，结果比较
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

'''
图像卷积操作自实现
    （opencv自带的卷积操作函数cv2.filter2D）
'''
def convolve(image, kernel):
    # 输入图像和核的尺寸
    (iH, iW) = image.shape[:2]
    (kH, kW) = kernel.shape[:2]

    # 选择pad，卷积后图像大小不变
    pad = (kW - 1) // 2
    # 重复最后一个元素，top, bottom, left, right
    image = cv2.copyMakeBorder(image, pad, pad, pad, pad,
        cv2.BORDER_REPLICATE)
    output = np.zeros((iH, iW), dtype="float32")

    # 卷积操作
    for y in np.arange(pad, iH + pad):
        for x in np.arange(pad, iW + pad):
            # 提取每一个卷积区域
            roi = image[y - pad:y + pad + 1, x - pad:x + pad + 1]
            # 内积运算
            k = (roi * kernel).sum()
            # 保存相应的结果
            output[y - pad, x - pad] = k

    output = np.clip(output,0,255)
    output = output.astype(np.uint8)
    # 将得到的结果放缩到[0, 255]
    # output = rescale_intensity(output, in_range=(0, 255))
    # output = (output * 255).astype("uint8")

    return output

# 分别构建两个卷积核
smallBlur = np.ones((7, 7), dtype="float") * (1.0 / (7 * 7))
largeBlur = np.ones((21, 21), dtype="float") * (1.0 / (21 * 21))

# 尝试不同的卷积核
sharpen = np.array((
    [0, -1, 0],
    [-1, 5, -1],
    [0, -1, 0]), dtype="int")

laplacian = np.array((
    [0, 1, 0],
    [1, -4, 1],
    [0, 1, 0]), dtype="int")


sobelX = np.array((
    [-1, 0, 1],
    [-2, 0, 2],
    [-1, 0, 1]), dtype="int")

sobelY = np.array((
    [-1, -2, -1],
    [0, 0, 0],
    [1, 2, 1]), dtype="int")

# 卷积核与对应名称集合
kernelBank = (
    ("small_blur", smallBlur),
    ("large_blur", largeBlur),
    ("sharpen", sharpen),
    ("laplacian", laplacian),
    ("sobel_x", sobelX),
    ("sobel_y", sobelY)
)


def main(args=None):                  # ROS2节点主入口main函数
    rclpy.init(args=args)             # ROS2 Python接口初始化
    node = Node("convolutions")            # 创建ROS2节点对象并进行初始化
    node.get_logger().info("convolutions node start !!!")

    bridge = CvBridge()

    baseDir = "/home/huanyu/robot_ws/src/opencv_course/"

    # 简单起见，用灰度图来玩
    image = cv2.imread(baseDir + "data/lena.jpg")
    if image is None:
        print("读取图片失败，请检查图片路径是否正确！")
        exit()
    else:
        print("读取图片成功！")
        print("image.shape:",image.shape)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    imgPub = node.create_publisher(Image, '/image/img_raw', 1) #创建图片话题发布器

    convoPubList = []
    # 遍历每一个核
    for (kernelName, kernel) in kernelBank:
        print("[INFO] applying {} kernel".format(kernelName))
        convoleOutput = convolve(gray, kernel) # 自实现的卷积操作函数
        # -1 表示深度一致
        opencvOutput = cv2.filter2D(gray, -1, kernel) # opencv自带的卷积操作函数
        # # 分别展示结果 
        # cv2.imshow("original", gray)
        # cv2.imshow("{} - convole".format(kernelName), convoleOutput)
        # cv2.imshow("{} - opencv".format(kernelName), opencvOutput)
        # cv2.waitKey(0)
        # cv2.destroyAllWindows()

        convoContract = np.hstack((convoleOutput, opencvOutput)) #将这两个卷积操作结果按行放在一起便于比较
        convPub = node.create_publisher(Image, '/image/%s'%kernelName, 1) #创建图片话题发布器
        convoPubList.append( (convoContract,convPub) )      

    count = 0
    while rclpy.ok():
        publishImage(gray, imgPub, bridge,"mono8")
        for convoContract,convPub in convoPubList:
            publishImage(convoContract, convPub, bridge,"mono8") #将上面图像操作的结果以图片的话题形式发布

        time.sleep(1)
        count += 1
        print("publish ",count)
        
    node.destroy_node()                  # 销毁节点对象
    rclpy.shutdown()                     # 关闭ROS2 Python接口


    # #遍历每一个核
    # for (kernelName, kernel) in kernelBank:
    #   print("[INFO] applying {} kernel".format(kernelName))
    #   convoleOutput = convolve(gray, kernel)
    #   # -1 表示深度一致
    #   opencvOutput = cv2.filter2D(gray, -1, kernel)




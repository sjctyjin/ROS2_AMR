#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
@作者: 古月居(www.guyuehome.com)
@说明: ROS2话题示例-发布图像话题
"""

import rclpy                        # ROS2 Python接口库
from rclpy.node import Node         # ROS2 节点类
from sensor_msgs.msg import CompressedImage,Image   # 图像消息类型
from cv_bridge import CvBridge      # ROS与OpenCV图像转换类
import cv2                          # Opencv图像处理库
import numpy as np

"""
创建一个发布者节点
"""
class ImagePublisher(Node):

    def __init__(self, name):
        super().__init__(name)                                           # ROS2节点父类初始化
        self.publisher_ = self.create_publisher(CompressedImage, 'image_raw/compressed', 10)
        self.subscription = self.create_subscription(
            Image,  # 如果您是從其他話題接收原始圖像
            '/camera/color/image_raw',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        # 圖像壓縮，這裡使用JPEG格式
        encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), 90]  # 設置JPEG質量
        result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
        if result:
            compressed_msg = CompressedImage()
            compressed_msg.header = msg.header  # 繼承原始圖像的header信息
            compressed_msg.format = "jpeg"
            compressed_msg.data = np.array(encimg).tobytes()
            self.publisher_.publish(compressed_msg)
            self.get_logger().info('Publishing compressed image')
        else:
            self.get_logger().error('Image compression failed')

def main(args=None):                                 # ROS2节点主入口main函数
    rclpy.init(args=args)                            # ROS2 Python接口初始化
    node = ImagePublisher("topic_webcam_sub_compress")        # 创建ROS2节点对象并进行初始化
    rclpy.spin(node)                                 # 循环等待ROS2退出
    node.destroy_node()                              # 销毁节点对象
    rclpy.shutdown()                                 # 关闭ROS2 Python接口

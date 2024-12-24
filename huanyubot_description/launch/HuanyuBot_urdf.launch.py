#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():

    urdf = os.path.join(         # 找到配置文件的完整路径
      get_package_share_directory('huanyubot_description'),
      'urdf',
      'huanyubot.urdf'
      )

    with open(urdf, 'r') as infp:
      robot_desc = infp.read()

    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        node_executable='robot_state_publisher',
        parameters=[{'robot_description': robot_desc}],
        arguments=[urdf]
    )

    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        node_executable='joint_state_publisher',
    )

    return LaunchDescription([
        joint_state_publisher_node,
        robot_state_publisher_node, 
    ])


#!/usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import LogInfo
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    #机器人URDF模型参数
    urdf = os.path.join(  
      get_package_share_directory('huanyubot_description'),
      'urdf',
      'huanyubot.urdf'
      )
    with open(urdf, 'r') as infp:
      robot_desc = infp.read()

    # 机器人底盘参数
    usart_port = LaunchConfiguration('usart_port', default='/dev/huanyu_base')
    baud_data = LaunchConfiguration('baud_data', default='115200') 
    robot_frame_id = LaunchConfiguration('robot_frame_id', default='base_footprint') 
    smoother_cmd_vel = LaunchConfiguration('smoother_cmd_vel', default='cmd_vel') 
    publish_odom = LaunchConfiguration('publish_odom', default='true') 

    filter_Vx_match = LaunchConfiguration('filter_Vx_match', default='1.0') 
    filter_Vy_match = LaunchConfiguration('filter_Vy_match', default='1.0') 
    filter_Vth_match = LaunchConfiguration('filter_Vth_match', default='1.0') 

    KP = LaunchConfiguration('KP', default='50.0') 
    KI = LaunchConfiguration('KI', default='100.0') 
    KD = LaunchConfiguration('KD', default='20.0') 

    #思岚A1雷达参数
    serial_port = LaunchConfiguration('serial_port', default='/dev/huanyu_laser')
    serial_baudrate = LaunchConfiguration('serial_baudrate', default='115200') #for A1/A2 is 115200
    frame_id = LaunchConfiguration('frame_id', default='laser')
    inverted = LaunchConfiguration('inverted', default='false')
    angle_compensate = LaunchConfiguration('angle_compensate', default='true')

    return LaunchDescription([
        # 静态坐标变换
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_link',
            output="screen",
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint','base_link' ] ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_gyro',
            output="screen",
            arguments=['0', '0', '0', '0', '0', '0', 'base_footprint','gyro_link' ] ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_laser',
            output="screen",
            arguments=['0.08', '0', '0.14', '3.1415', '0', '0', 'base_footprint','laser' ] ),

        # 加载机器人URDF模型
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            arguments=[urdf]),
        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher'),

        # 机器人底盘节点及参数
        DeclareLaunchArgument( 'usart_port', default_value=usart_port),
        DeclareLaunchArgument( 'baud_data', default_value=baud_data),
        DeclareLaunchArgument( 'robot_frame_id', default_value=robot_frame_id),
        DeclareLaunchArgument( 'smoother_cmd_vel', default_value=smoother_cmd_vel),
        DeclareLaunchArgument( 'publish_odom', default_value=publish_odom),
        DeclareLaunchArgument( 'filter_Vx_match', default_value=filter_Vx_match),
        DeclareLaunchArgument( 'filter_Vy_match', default_value=filter_Vy_match),
        DeclareLaunchArgument( 'filter_Vth_match', default_value=filter_Vth_match),
        DeclareLaunchArgument( 'KP', default_value=KP),
        DeclareLaunchArgument( 'KI', default_value=KI),
        DeclareLaunchArgument( 'KD', default_value=KD),
        Node(
            package='huanyu_robot_start',
            executable='huanyu_robot_node',
            name='Robot_start_node',
            parameters=[{'usart_port': usart_port, 
                         'baud_data': baud_data, 
                         'robot_frame_id': robot_frame_id,
                         'smoother_cmd_vel': smoother_cmd_vel,
                         'publish_odom': publish_odom,

                         'filter_Vx_match': filter_Vx_match,
                         'filter_Vy_match': filter_Vy_match,
                         'filter_Vth_match': filter_Vth_match,

                         'KP': KP,
                         'KI': KI,
                         'KD': KD }],
            output='screen'),

        #思岚A1参数及节点
        DeclareLaunchArgument(
            'serial_port', default_value=serial_port,
            description='Specifying usb port to connected lidar'),
        DeclareLaunchArgument(
            'serial_baudrate', default_value=serial_baudrate,
            description='Specifying usb port baudrate to connected lidar'),        
        DeclareLaunchArgument(
            'frame_id', default_value=frame_id,
            description='Specifying frame_id of lidar'),
        DeclareLaunchArgument(
            'inverted', default_value=inverted,
            description='Specifying whether or not to invert scan data'),
        DeclareLaunchArgument(
            'angle_compensate', default_value=angle_compensate,
            description='Specifying whether or not to enable angle_compensate of scan data'),
        Node(
            package='sllidar_ros2',
            executable='sllidar_node',
            name='sllidar_node',
            parameters=[{'serial_port': serial_port, 
                         'serial_baudrate': serial_baudrate, 
                         'frame_id': frame_id,
                         'inverted': inverted, 
                         'angle_compensate': angle_compensate}],
            output='screen'),
    ])


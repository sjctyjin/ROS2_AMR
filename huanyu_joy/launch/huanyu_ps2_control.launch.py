#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    joystick_device = LaunchConfiguration('joystick_device', default='/dev/input/js0')
    maxLinear_x = LaunchConfiguration('maxLinear_x', default='0.3') 
    maxLinear_y = LaunchConfiguration('maxLinear_y', default='0.3') 
    maxAngular_z = LaunchConfiguration('maxAngular_z', default='1.5') 

    return LaunchDescription([

        DeclareLaunchArgument(
            'joystick_device', 
            default_value=joystick_device),

        DeclareLaunchArgument(
            'maxLinear_x',
            default_value=maxLinear_x),

         DeclareLaunchArgument(
            'maxLinear_y',
            default_value=maxLinear_y),

         DeclareLaunchArgument(
            'maxAngular_z',
            default_value=maxAngular_z),
 

        Node(
            package='huanyu_joy',
            executable='huanyu_joy_node',
            name='huanyu_joy_node',
            output='screen',
            parameters=[{'joystick_device': joystick_device, 
                         'maxLinear_x': maxLinear_x, 
                         'maxLinear_y': maxLinear_y,
                         'maxAngular_z': maxAngular_z}],
            ),
    ])


#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition

def generate_launch_description():
    # 声明启动参数
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='640',
        description='Image width'
    )
    
    height_arg = DeclareLaunchArgument(
        'height', 
        default_value='480',
        description='Image height'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='30', 
        description='Frames per second'
    )
    
    enable_depth_arg = DeclareLaunchArgument(
        'enable_depth',
        default_value='false',
        description='Enable depth stream'
    )
    
    camera_info_url_arg = DeclareLaunchArgument(
        'camera_info_url',
        default_value='',
        description='Camera calibration file URL'
    )
    
    # D435发布节点
    d435_node = Node(
        package='d435_publisher',
        executable='d435_publisher_node',
        name='d435_publisher',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'), 
            'fps': LaunchConfiguration('fps'),
            'enable_depth': LaunchConfiguration('enable_depth'),
            'camera_info_url': LaunchConfiguration('camera_info_url')
        }]
    )
    
    return LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        enable_depth_arg,
        camera_info_url_arg,
        d435_node
    ])
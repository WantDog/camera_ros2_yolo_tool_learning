#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    专门用于相机标定的启动文件
    使用高分辨率以获得更好的标定精度
    """
    
    # 声明启动参数 - 标定推荐使用高分辨率
    width_arg = DeclareLaunchArgument(
        'width',
        default_value='1280',  # 标定推荐使用更高分辨率
        description='Image width for calibration'
    )
    
    height_arg = DeclareLaunchArgument(
        'height', 
        default_value='720',   # 标定推荐使用更高分辨率
        description='Image height for calibration'
    )
    
    fps_arg = DeclareLaunchArgument(
        'fps',
        default_value='15',    # 标定时可以降低帧率
        description='Frames per second'
    )
    
    # D435发布节点 - 标定配置
    d435_calibration_node = Node(
        package='d435_publisher',
        executable='d435_publisher_node',
        name='d435_calibration_publisher',
        output='screen',
        parameters=[{
            'width': LaunchConfiguration('width'),
            'height': LaunchConfiguration('height'), 
            'fps': LaunchConfiguration('fps'),
            'enable_depth': False,  # 标定时通常只需要彩色图像
            'camera_info_url': ''
        }],
        remappings=[
            ('/d435/color/image_raw', '/camera/image_raw'),
            ('/d435/color/camera_info', '/camera/camera_info')
        ]
    )
    
    return LaunchDescription([
        width_arg,
        height_arg,
        fps_arg,
        d435_calibration_node
    ])
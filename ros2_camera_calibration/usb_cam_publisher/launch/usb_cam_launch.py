from launch import LaunchDescription
from launch_ros.actions import Node
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    cam_type = LaunchConfiguration('cam_type')

    usb_cam_node = Node(
        package='usb_cam_publisher',
        executable='usb_cam_publisher',
        name='usb_cam_publisher',
        output='screen'
    )
    return LaunchDescription([
        usb_cam_node
    ])

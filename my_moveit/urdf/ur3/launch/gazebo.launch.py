from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('ur3')
    urdf_file = os.path.join(pkg_share, 'urdf', 'ur3.urdf')

    return LaunchDescription([

        # 启动 Gazebo 空世界
        ExecuteProcess(
            cmd=['gazebo', '--verbose', '-s', 'libgazebo_ros_factory.so'],
            output='screen'
        ),

        # 静态 TF：base_link -> base_footprint
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='tf_footprint_base',
            arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
        ),

        # 在 Gazebo 中生成 UR3 模型
        Node(
            package='gazebo_ros',
            executable='spawn_entity.py',
            arguments=['-file', urdf_file, '-entity', 'ur3'],
            output='screen'
        ),

        # 模拟关节校准 (ROS 1 里的 fake_joint_calibration)
        Node(
            package='rclcpp_components',  # 或者自写节点，这里只是替代思路
            executable='component_container',
            name='fake_joint_calibration',
            arguments=[],
            output='screen'
        )
        # ⚠️ 注意：ROS 2 没有 rostopic pub，可以写一个小 Python 节点，发布 /calibrated std_msgs/Bool(True)
    ])


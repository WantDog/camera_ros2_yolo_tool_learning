from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # 声明模型文件参数
    model_arg = DeclareLaunchArgument(
        name='model',
        default_value='arm_five.urdf',
        description='URDF 模型文件'
    )

    # 获取 URDF 文件路径
    pkg_share = FindPackageShare('arm_five')
    urdf_path = PathJoinSubstitution([pkg_share, 'urdf', LaunchConfiguration('model')])

    # robot_state_publisher 节点
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': Command(['xacro ', urdf_path])
        }]
    )

    # joint_state_publisher_gui 节点
    joint_state_publisher_node = Node(
        package='joint_state_publisher_gui',
        executable='joint_state_publisher_gui',
        name='joint_state_publisher_gui',
        output='screen'
    )

    # rviz 节点
    rviz_config_path = PathJoinSubstitution([pkg_share, 'urdf.rviz'])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config_path],
        output='screen'
    )

    return LaunchDescription([
        model_arg,
        joint_state_publisher_node,
        robot_state_publisher_node,
        rviz_node
    ])

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    # 获取包路径
    gazebo_pkg = FindPackageShare('gazebo_ros')
    arm_five_pkg = FindPackageShare('arm_five')

    # 获取 urdf 模型路径
    urdf_path = PathJoinSubstitution([arm_five_pkg, 'urdf', 'arm_five.urdf'])

    # 启动 Gazebo 空世界
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([gazebo_pkg, 'launch', 'gazebo.launch.py'])
        )
    )

    # 静态 TF 发布器 (base_footprint -> base_link)
    static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='tf_footprint_base',
        arguments=['0', '0', '0', '0', '0', '0', 'base_link', 'base_footprint']
    )

    # 生成模型到 Gazebo 中
    spawn_model = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        arguments=[
            '-file', urdf_path,
            '-entity', 'arm_five'
        ],
        output='screen'
    )

    # 模拟标定话题（可选）
    fake_joint_calibration = ExecuteProcess(
        cmd=['ros2', 'topic', 'pub', '/calibrated', 'std_msgs/msg/Bool', 'data:true'],
        shell=True
    )

    return LaunchDescription([
        gazebo_launch,
        static_tf,
        spawn_model,
        fake_joint_calibration
    ])

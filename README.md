# ROS2机器人视觉与运动控制工程

集成多种相机驱动、机械臂运动规划和YOLO目标检测的ROS2综合项目，专为机器人视觉抓取和工业自动化应用设计。

## 功能模块

- **相机驱动**: Intel D435i深度相机、海康威视工业相机、USB相机
- **机械臂控制**: 基于MoveIt2的运动规划和轨迹执行  
- **视觉处理**: YOLO目标检测和数据集处理工具
- **相机标定**: 完整的相机标定流程

## 项目结构

```
├── D435i/                      # Intel D435i相机驱动
├── ros2_camera_calibration/    # 相机标定工具集
├── my_moveit/                  # MoveIt2机械臂控制
└── yolo_tool/                  # YOLO数据集处理工具
```

## 环境要求

- Ubuntu 22.04 LTS + ROS2 Humble
- Intel RealSense SDK 2.0
- MoveIt2 + OpenCV 4.7.0+

## 快速安装

```bash
# 安装依赖
sudo apt install ros-humble-desktop ros-humble-moveit
sudo apt install librealsense2-dev ros-humble-camera-calibration

# 编译项目
mkdir -p ~/robot_ws/src && cd ~/robot_ws/src
git clone <your-repo-url> .
cd ~/robot_ws && colcon build && source install/setup.bash
```

## 使用说明

各模块详细使用说明请参考对应目录下的README文档：

- [相机标定指南](ros2_camera_calibration/README.md)
- [MoveIt2开发经验](my_moveit/机械臂开发经验.md)  
- [YOLO工具使用](yolo_tool/数据集处理/README.md)
# ROS2 相机标定 :dagger:

本文档整合了 Intel RealSense D435 相机、USB 相机和海康威视相机的标定指南和配置信息。



###### ***使用该仓库 ，如果没有ros2标定了解的话，建议去阅读下 https://www.ncnynl.com/archives/202110/4707.html再使用本代码.***

**提醒：下面使用教程为ai生成，仅供参考**

## 📋 目录

1. [Intel RealSense D435 相机标定](#intel-realsense-d435-相机标定)

2. [USB 相机标定](#usb-相机标定)

3. [海康威视相机标定](#海康威视相机标定)

   

---

## Intel RealSense D435 相机标定

### 🚀 快速开始

#### 1. 系统准备

```bash
# 安装Intel RealSense SDK 2.0
sudo apt-get update
sudo apt-get install librealsense2-dev librealsense2-utils

# 安装Python RealSense库 (用于测试脚本)
pip3 install pyrealsense2

# 验证安装
realsense-viewer
```

#### 2. 编译工作空间

```bash
# 编译D435发布包
colcon build --packages-select d435_publisher
source install/setup.bash
```

#### 3. 测试D435连接

```bash
# 运行连接测试脚本
python3 d435_publisher/scripts/test_d435_connection.py
```

### 📷 D435 相机标定流程

#### 步骤1: 启动D435发布节点

```bash
# 启动标定专用配置 (1280x720@15fps)
ros2 launch d435_publisher d435_calibration_launch.py

# 或者自定义分辨率
ros2 launch d435_publisher d435_calibration_launch.py width:=1920 height:=1080 fps:=10
```

#### 步骤2: 查看图像流

```bash
# 在新终端中查看图像
ros2 run rqt_image_view rqt_image_view /camera/image_raw

# 或使用rviz2
rviz2
# 在rviz2中添加Image显示，话题选择 /camera/image_raw
```

#### 步骤3: 进行相机标定

##### 方法1: 使用ROS2 camera_calibration包

```bash
# 安装标定工具 (如果未安装)
sudo apt-get install ros-humble-camera-calibration

# 运行标定 (8x6棋盘格，方格大小10.8cm)
ros2 run camera_calibration cameracalibrator \
    --size 8x6 \
    --square 0.108 \
    image:=/camera/image_raw \
    camera:=/camera
```

##### 方法2: 使用image_pipeline标定

```bash
# 安装image_pipeline
sudo apt-get install ros-humble-image-pipeline

# 运行标定
ros2 launch camera_calibration calibrate.launch.py \
    image:=/camera/image_raw \
    camera:=/camera
```

### 🔧 D435 参数配置

#### 支持的分辨率和帧率

| 分辨率 | 最大帧率 | 推荐用途 |
|--------|----------|----------|
| 424x240 | 60fps | 实时应用 |
| 640x480 | 60fps | 标准应用 |
| 848x480 | 60fps | 宽屏应用 |
| 1280x720 | 30fps | 高精度标定 |
| 1920x1080 | 15fps | 最高精度标定 |

#### 标定推荐配置

```yaml
# 高精度标定配置
width: 1280
height: 720
fps: 15
enable_depth: false  # 标定时只需要彩色图像
```

### 📊 D435 发布的话题

#### 标准模式
- `/d435/color/image_raw` - 彩色图像
- `/d435/color/camera_info` - 彩色相机信息
- `/d435/depth/image_raw` - 深度图像 (可选)
- `/d435/depth/camera_info` - 深度相机信息 (可选)

#### 标定模式 (重映射后)
- `/camera/image_raw` - 标定用彩色图像
- `/camera/camera_info` - 标定用相机信息

---

## USB 相机标定

### 启动 USB 相机节点

```bash
# 方法1: 使用启动文件
ros2 launch usb_cam_publisher usb_cam_launch.py

# 方法2: 直接运行节点
ros2 run usb_cam_publisher usb_cam_publisher
```

### 查看相机画面

```bash
ros2 run rqt_image_view rqt_image_view
```

### USB 相机标定命令

```bash
ros2 run camera_calibration cameracalibrator \
    --size 8x11 \
    --square 0.04 \
    --ros-args --remap /image:=/image_raw \
    --ros-args --remap camera:=/custom_camera
```

### USB 相机配置

- **设备**: `/dev/video2`
- **分辨率**: 1280x720
- **帧率**: 30fps

---

## 海康威视相机标定

### 启动海康威视相机节点

```bash
source ~/.bashrc
source install/setup.bash
ros2 run hk_cam_node hk_cam_node
```

### ⚠️ 重要提醒

**记得将HK相机的配置文件路径换一下！！！**

海康威视相机使用配置文件：`camera_driver/camera_init/HIKcamera0.yaml`

---

## 包结构

- **camera_driver**: 海康威视相机驱动库
- **hk_cam_node**: 海康威视相机的 ROS2 节点
- **usb_cam_publisher**: 通用 USB 相机的 ROS2 节点
- **d435_publisher**: Intel RealSense D435 相机的 ROS2 节点

---

## 构建说明

```bash
# 设置 ROS2 环境
source /opt/ros/humble/setup.bash

# 构建工作空间
colcon build

# 设置工作空间
source install/setup.bash
```

## 🎯 标定技巧

### 1. 环境准备
- 充足均匀的光照
- 避免反光和阴影
- 稳定的相机安装

### 2. 标定板要求
- 高对比度的棋盘格
- 平整不弯曲
- 适当的大小 (占图像1/4到1/2)

### 3. 图像采集
- 多角度、多位置
- 覆盖图像边缘区域
- 避免模糊和过曝

### 4. 质量评估
- 重投影误差 < 0.5像素
- 检测成功率 > 90%
- 畸变矫正效果良好

---

## 🛠️ 故障排除

### 1. 找不到设备

```bash
# 检查USB连接
lsusb | grep Intel  # 对于D435
lsusb               # 查看所有USB设备

# 检查RealSense设备
realsense-viewer

# 检查权限
sudo usermod -a -G dialout $USER
# 需要重新登录
```

### 2. 编译错误

```bash
# 确保安装了依赖
sudo apt-get install librealsense2-dev librealsense2-utils

# 检查ROS2环境
source /opt/ros/humble/setup.bash
```

### 3. 运行时错误

```bash
# 重启udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 检查RealSense服务
sudo systemctl status realsense
```

---

## 📚 相关资源

- [Intel RealSense官方文档](https://dev.intelrealsense.com/)
- [ROS2 camera_calibration教程](https://docs.ros.org/en/humble/p/camera_calibration/)
- [OpenCV相机标定指南](https://docs.opencv.org/4.x/dc/dbb/tutorial_py_calibration.html)

---

## 📝 注意事项

- 确保D435相机连接到USB 3.0端口以获得最佳性能
- 标定过程中保持相机稳定，避免移动
- OpenCV版本警告是正常的，不会影响功能
- 确保相机设备具有适当的权限
- 对于海康威视相机，确保专有驱动库可用
- 海康威视相机配置文件路径需要根据实际情况调整

---

*本文档整合了三个相机系统的完整标定指南，为ROS2环境下的相机标定提供全面的参考。*
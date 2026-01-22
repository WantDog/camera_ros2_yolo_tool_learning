# D435 Publisher

Intel RealSense D435 相机图像发布节点，专门用于相机标定。

## 功能特性

- 发布D435彩色图像流
- 可选发布深度图像流
- 自动获取并发布相机内参信息
- 支持多种分辨率和帧率配置
- 专门的标定模式配置

## 依赖项

### 系统依赖
```bash
# 安装Intel RealSense SDK 2.0
sudo apt-get update
sudo apt-get install librealsense2-dev librealsense2-utils

# 验证安装
realsense-viewer
```

### ROS2依赖
- rclcpp
- sensor_msgs
- cv_bridge
- image_transport
- camera_info_manager

## 编译

```bash
# 在工作空间根目录
colcon build --packages-select d435_publisher
source install/setup.bash
```

## 使用方法

### 1. 基本使用
```bash
# 启动D435发布节点 (默认640x480@30fps)
ros2 launch d435_publisher d435_launch.py

# 自定义分辨率和帧率
ros2 launch d435_publisher d435_launch.py width:=1280 height:=720 fps:=15

# 启用深度流
ros2 launch d435_publisher d435_launch.py enable_depth:=true
```

### 2. 相机标定专用
```bash
# 使用标定专用配置 (1280x720@15fps)
ros2 launch d435_publisher d435_calibration_launch.py

# 查看图像流
ros2 run rqt_image_view rqt_image_view /camera/image_raw
```

### 3. 直接运行节点
```bash
# 使用默认参数
ros2 run d435_publisher d435_publisher_node

# 使用参数文件
ros2 run d435_publisher d435_publisher_node --ros-args --params-file src/d435_publisher/config/d435_params.yaml
```

## 话题说明

### 发布的话题
- `/d435/color/image_raw` (sensor_msgs/Image) - 彩色图像
- `/d435/color/camera_info` (sensor_msgs/CameraInfo) - 彩色相机信息
- `/d435/depth/image_raw` (sensor_msgs/Image) - 深度图像 (可选)
- `/d435/depth/camera_info` (sensor_msgs/CameraInfo) - 深度相机信息 (可选)

### 标定模式重映射
标定启动文件会将话题重映射为标准的标定话题名称：
- `/camera/image_raw` - 标定用彩色图像
- `/camera/camera_info` - 标定用相机信息

## 参数说明

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| width | int | 640 | 图像宽度 |
| height | int | 480 | 图像高度 |
| fps | int | 30 | 帧率 |
| enable_depth | bool | false | 是否启用深度流 |
| camera_info_url | string | "" | 相机标定文件路径 |

## 支持的分辨率

| 分辨率 | 最大帧率 | 用途 |
|--------|----------|------|
| 424x240 | 60fps | 低延迟应用 |
| 640x480 | 60fps | 标准应用 |
| 848x480 | 60fps | 宽屏应用 |
| 1280x720 | 30fps | 高精度标定 |

## 相机标定流程

1. 启动标定模式：
```bash
ros2 launch d435_publisher d435_calibration_launch.py
```

2. 使用ROS2标定工具：
```bash
ros2 run camera_calibration cameracalibrator --size 8x6 --square 0.108 image:=/camera/image_raw camera:=/camera
```

3. 或使用image_pipeline标定：
```bash
ros2 launch camera_calibration calibrate.launch.py
```

## 故障排除

### 1. 找不到相机设备
```bash
# 检查设备连接
lsusb | grep Intel
realsense-viewer

# 检查权限
sudo usermod -a -G dialout $USER
# 重新登录后生效
```

### 2. 编译错误
```bash
# 确保安装了librealsense2-dev
sudo apt-get install librealsense2-dev

# 检查OpenCV版本
pkg-config --modversion opencv4
```

### 3. 运行时错误
```bash
# 检查RealSense服务
sudo systemctl status realsense

# 重启udev规则
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## 注意事项

1. 标定时建议使用高分辨率 (1280x720) 以获得更好的精度
2. 标定过程中保持相机稳定，避免自动对焦干扰
3. 使用足够大的标定板，确保在图像中占据合适比例
4. 标定时采集多个角度和位置的图像 (建议20-30张)
5. 确保标定板平整，避免弯曲影响标定精度
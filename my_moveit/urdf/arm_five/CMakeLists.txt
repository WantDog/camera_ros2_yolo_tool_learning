cmake_minimum_required(VERSION 3.5)
project(arm_five)

# 1. 找到必要的依赖
find_package(ament_cmake REQUIRED)

# 2. 安装资源文件（与 ROS 1 类似）
install(
  DIRECTORY launch urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

# 3. 导出包信息
ament_package()

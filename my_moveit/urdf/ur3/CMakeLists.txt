cmake_minimum_required(VERSION 3.5)
project(ur3)

# 使用 ament_cmake
find_package(ament_cmake REQUIRED)

# 如果只是纯描述性包（urdf、launch、meshes），一般不需要依赖
# 但如果要用 xacro、robot_state_publisher 等，可以加 find_package()

# 安装配置文件、launch、urdf、meshes
install(
  DIRECTORY config launch meshes urdf
  DESTINATION share/${PROJECT_NAME}
)

ament_package()


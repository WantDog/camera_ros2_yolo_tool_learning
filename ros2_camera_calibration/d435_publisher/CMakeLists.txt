cmake_minimum_required(VERSION 3.8)
project(d435_publisher)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(camera_info_manager REQUIRED)
find_package(std_msgs REQUIRED)

# Find RealSense2 library
find_package(realsense2 REQUIRED)

# Find OpenCV
find_package(OpenCV REQUIRED)

# Add executable
add_executable(d435_publisher_node src/d435_publisher_node.cpp)

# Include directories
target_include_directories(d435_publisher_node PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${OpenCV_INCLUDE_DIRS}
)

# Link libraries
target_link_libraries(d435_publisher_node
  ${realsense2_LIBRARY}
  ${OpenCV_LIBRARIES}
)

# ROS2 dependencies
ament_target_dependencies(d435_publisher_node
  rclcpp
  sensor_msgs
  cv_bridge
  camera_info_manager
  std_msgs
)

# Install targets
install(TARGETS d435_publisher_node
  DESTINATION lib/${PROJECT_NAME})

# Install launch files
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch)

# Install config files
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
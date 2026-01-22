cmake_minimum_required(VERSION 3.8)
project(hk_cam_node)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)
find_package(camera_driver REQUIRED)
find_package(PkgConfig REQUIRED)

# Find TIFF library to resolve GDAL linking issues
pkg_check_modules(TIFF REQUIRED libtiff-4)


add_executable(hk_cam_node src/hk_cam_node.cpp)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

target_include_directories(hk_cam_node PUBLIC
  ${OpenCV_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
  ${TIFF_INCLUDE_DIRS}
)

ament_target_dependencies(hk_cam_node
  rclcpp
  sensor_msgs
  cv_bridge
)

target_link_libraries(hk_cam_node
  ${OpenCV_LIBS}
  camera_driver::camera_driver
  ${TIFF_LIBRARIES}
)

install(TARGETS
  hk_cam_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()

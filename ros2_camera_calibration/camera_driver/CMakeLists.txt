cmake_minimum_required(VERSION 3.8)

project(camera_driver)

# Default to C++17
if(NOT CMAKE_CXX_STANDARD)
	set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
	add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(OpenCV REQUIRED)

# Include directories
include_directories(
	${PROJECT_SOURCE_DIR}/include
	${OpenCV_INCLUDE_DIRS}
)

# Create the library
add_library(${PROJECT_NAME}
	src/camera_driver.cpp
)

# Link libraries
target_link_libraries(${PROJECT_NAME} PUBLIC
	${CMAKE_CURRENT_SOURCE_DIR}/lib/libMvCameraControl.so
	${OpenCV_LIBS}
	${rclcpp_LIBRARIES}
	${cv_bridge_LIBRARIES}
)

# Link directories
target_link_directories(${PROJECT_NAME} PRIVATE
	${CMAKE_CURRENT_BINARY_DIR}/lib
)

# Install targets
install(TARGETS ${PROJECT_NAME}
	EXPORT export_${PROJECT_NAME}
	LIBRARY DESTINATION lib
	ARCHIVE DESTINATION lib
	RUNTIME DESTINATION lib/${PROJECT_NAME}
	INCLUDES DESTINATION include
)

# Install headers
install(DIRECTORY include/
	DESTINATION include
)

# Install library files
install(FILES ${CMAKE_CURRENT_SOURCE_DIR}/lib/libMvCameraControl.so
	DESTINATION lib
)

# Export targets
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(rclcpp std_msgs sensor_msgs cv_bridge)
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})

ament_package()

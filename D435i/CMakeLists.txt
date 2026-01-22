cmake_minimum_required(VERSION 3.10)
project(d435i)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

add_executable(main camera.cpp)
target_link_libraries(main ${OpenCV_LIBS} realsense2)
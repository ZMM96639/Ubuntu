cmake_minimum_required(VERSION 3.19)
project(camera_calibration)
set(CMAKE_BUILD_TYPE Debug)
find_package(OpenCV REQUIRED)

add_executable(camera_calibration main.cpp)

target_link_libraries(camera_calibration ${OpenCV_LIBS})
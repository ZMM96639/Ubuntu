cmake_minimum_required(VERSION 3.20)
project(camera)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(camera sgbm.cpp)
target_link_libraries(camera ${OpenCV_LIBS})

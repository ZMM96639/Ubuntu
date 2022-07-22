cmake_minimum_required(VERSION 3.19)
project(SURF)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(SURF main.cpp)

target_link_libraries(SURF ${OpenCV_LIBS})
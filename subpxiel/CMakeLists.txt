cmake_minimum_required(VERSION 3.19)
project(subpxiel)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(subpxiel main.cpp)

target_link_libraries(subpxiel ${OpenCV_LIBS})
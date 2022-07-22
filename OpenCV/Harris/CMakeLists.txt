cmake_minimum_required(VERSION 3.19)
project(Harris)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(Harris main.cpp)
target_link_libraries(Harris ${OpenCV_LIBS})
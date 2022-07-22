cmake_minimum_required(VERSION 3.19)
project(Harris_sample)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)

add_executable(Harris_sample main.cpp)
target_link_libraries(Harris_sample ${OpenCV_LIBS})
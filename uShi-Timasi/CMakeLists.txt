cmake_minimum_required(VERSION 3.19)
project(uShi_Timasi)

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV REQUIRED)

add_executable(uShi_Timasi main.cpp)

target_link_libraries(uShi_Timasi ${OpenCV_LIBS})

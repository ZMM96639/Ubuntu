cmake_minimum_required(VERSION 3.20)
project(sgbm)

set(CMAKE_CXX_STANDARD 14)
include_directories(/usr/include/eigen3)
find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
set(DIR ${OpenCV_LIBS}
        ${Pangolin_LIBRARIES})
add_executable(sgbm main.cpp)
target_link_libraries(sgbm ${DIR})
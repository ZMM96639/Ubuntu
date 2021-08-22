cmake_minimum_required(VERSION 3.20)
project(SGBM)

set(CMAKE_CXX_STANDARD 14)
find_package(OpenCV REQUIRED)
include_directories(include)
set(SRC
        src/main.cpp
        src/SemiGlobalMatching.cpp
        src/sgm_util.cpp
        src/stdafx.cpp)

add_executable(SGBM ${SRC})
target_link_libraries(SGBM ${OpenCV_LIBS})




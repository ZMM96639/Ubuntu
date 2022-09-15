cmake_minimum_required(VERSION 3.0)
project(solvePnP)

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

include_directories(include)
aux_source_directory(src SRC)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS})


add_executable(pose_estimation Examples/pose_estimation.cpp)
target_link_libraries(pose_estimation ${PROJECT_NAME})

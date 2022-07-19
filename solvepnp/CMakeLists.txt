cmake_minimum_required(VERSION 3.0)
project(solvePnP)

find_package(OpenCV 4.5.0 REQUIRED)
find_package(Sophus REQUIRED)

include_directories(${EIGEN3_INCLUDE_DIRS})


add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS} ${Sophus_LIBRARIES})
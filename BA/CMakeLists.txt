cmake_minimum_required(VERSION 2.8)
project(bundleAdjustment)

set(CMAKE_BUILD_TYPE "Debug")

set(CMAKE_CXX_STANDARD 14)

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories(${OpenCV_LIBRARY_DIRS})

find_package(Sophus REQUIRED)
include_directories(${SOPHUS_INCLUDE_DIRS})
link_directories(${SOPHUS_LIBRARY_DIRS})

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(Fmt REQUIRED)
include_directories(${FMT_INCLUDE_DIRS})
link_directories(${FMT_LIBRARY_DIRS})


set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

include_directories(include Thirdparty/g2o)

aux_source_directory(src SRC)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME}
                      ${OpenCV_LIBS}
                      ${SOPHUS_LIBRARIES}
                      ${PROJECT_SOURCE_DIR}/Thirdparty/g2o/lib/libg2o.so
                      fmt)


add_executable(pose_estimation_3d2d Examples/pose_estimation_3d2d.cpp)
target_link_libraries(pose_estimation_3d2d ${PROJECT_NAME})
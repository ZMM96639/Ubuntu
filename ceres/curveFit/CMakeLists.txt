cmake_minimum_required(VERSION 3.0)

project(curvefitting)

find_package(OpenCV 4.5.0 REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})
link_directories((${OpencCV_LIBRARY_DIRS}))

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIRS})
link_directories(${EIGEN3_LIBRARY_DIRS})

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIRS})
link_directories(${CERES_LIBRARY_DIRS})

include_directories(${PROJECT_SOURCE_DIR}/include)

aux_source_directory(src SRC)

add_library(${PROJECT_NAME} ${SRC})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBS} ${CERES_LIBRARIES})

add_executable(curve Examples/curve.cpp)
target_link_libraries(curve ${PROJECT_NAME})
